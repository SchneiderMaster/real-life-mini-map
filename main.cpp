#include <exception>
#include <iostream>
#include <vector>
#include <algorithm>
#include <cmath>
#include <thread>
#include <mutex>
#include <atomic>

#include <osmium/io/any_input.hpp>
#include <osmium/geom/haversine.hpp>
#include <osmium/visitor.hpp>
#include <osmium/osm/location.hpp>
#include <osmium/handler.hpp>

#include <X11/Xlib.h>
#include <X11/Xutil.h>
#include <unistd.h>

#define WINDOW_HEIGHT 800
#define WINDOW_WIDTH 800

struct Point {
    int x, y;
};

struct Position {
    float x, y;
};

// --- GLOBALE DATENSTRUKTUREN ---
struct CachedWay {
    uint32_t start_index = 0;
    uint16_t node_count = 0;
    uint8_t tagMap = 0;
};

// Globaler Cache geschützt durch Mutex
std::vector<Position> global_node_pool;
std::vector<CachedWay> map_cache;
std::mutex cache_mutex;
std::atomic<bool> is_updating{false};

struct NodeEntry {
    osmium::object_id_type id;
    osmium::Location location;
    bool operator<(const NodeEntry& other) const { return id < other.id; }
};

// --- HANDLER ---

struct MySmartLocationHandler : public osmium::handler::Handler {
    std::vector<NodeEntry> local_index;
    osmium::Box &filter_box;

    MySmartLocationHandler(osmium::Box &box) : filter_box(box) {}

    void node(const osmium::Node &node) {
        if (filter_box.contains(node.location())) {
            local_index.push_back({node.id(), node.location()});
        }
    }

    void sort_index() {
        std::sort(local_index.begin(), local_index.end());
    }

    void way(osmium::Way &way) {
        for (auto &node_ref : way.nodes()) {
            auto it = std::lower_bound(local_index.begin(), local_index.end(), NodeEntry{node_ref.ref(), {0, 0}});
            if (it != local_index.end() && it->id == node_ref.ref()) {
                node_ref.set_location(it->location);
            }
        }
    }
};

struct TestHandler : public osmium::handler::Handler {
    std::vector<CachedWay> &local_cache;
    std::vector<Position> &local_pool;
    osmium::Box bounding_box;

    TestHandler(std::vector<CachedWay> &cache_out, std::vector<Position> &pool_out, double center_lat, double center_lon, double radius_m)
        : local_cache(cache_out), local_pool(pool_out) {
        const double earth_radius = 6371000.0;
        const double M_PI_CONST = 3.14159265358979323846;
        double lat_offset = (radius_m / earth_radius) * (180.0 / M_PI_CONST);
        double lon_offset = (radius_m / (earth_radius * std::cos(center_lat * M_PI_CONST / 180.0))) * (180.0 / M_PI_CONST);
        this->bounding_box = osmium::Box(osmium::Location{center_lon - lon_offset, center_lat - lat_offset}, 
                                         osmium::Location{center_lon + lon_offset, center_lat + lat_offset});
    }

    void way(const osmium::Way &way) {
        const char *highway = way.tags()["highway"];
        const char *waterway = way.tags()["waterway"];
        if (highway || waterway) {
            bool any_node_in_box = false;
            for (const auto& node : way.nodes()) {
                if (node.location().valid() && bounding_box.contains(node.location())) {
                    any_node_in_box = true;
                    break;
                }
            }

            if (any_node_in_box) {
                CachedWay c;
                c.start_index = static_cast<uint32_t>(local_pool.size());
                for (const auto &node : way.nodes()) {
                    if (node.location().valid()) {
                        local_pool.push_back({static_cast<float>(node.lon()), static_cast<float>(node.lat())});
                        c.node_count++;
                    }
                }
                if (way.tags().has_tag("highway", "residential") || way.tags().has_tag("highway", "secondary") || 
                    way.tags().has_tag("highway", "tertiary") || way.tags().has_tag("highway", "living_street")) {
                    c.tagMap |= 1;
                } else if (highway) {
                    c.tagMap |= 1 << 1;
                } else if (way.tags().has_tag("waterway", "river")) {
                    c.tagMap |= 1 << 7;
                }
                local_cache.push_back(std::move(c));
            }
        }
    }
};

// --- THREAD LOGIK ---

void backgroundUpdateTask(Position p, double radius) {
    is_updating = true;
    std::cout << "[Thread] Starting background update..." << std::endl;

    std::vector<Position> thread_pool;
    std::vector<CachedWay> thread_cache;

    // Erstelle Box für den Handler
    const double earth_radius = 6371000.0;
    const double M_PI_CONST = 3.14159265358979323846;
    double lat_offset = (radius / earth_radius) * (180.0 / M_PI_CONST);
    double lon_offset = (radius / (earth_radius * std::cos(p.x * M_PI_CONST / 180.0))) * (180.0 / M_PI_CONST);
    osmium::Box thread_box(osmium::Location{p.y - lon_offset, p.x - lat_offset}, 
                           osmium::Location{p.y + lon_offset, p.x + lat_offset});

    MySmartLocationHandler loc_handler{thread_box};
    TestHandler test_handler{thread_cache, thread_pool, p.x, p.y, radius};

    try {
        osmium::io::Reader reader{"./data/sh-map.pbf", osmium::osm_entity_bits::node | osmium::osm_entity_bits::way};
        bool indexed = false;
        while (osmium::memory::Buffer buffer = reader.read()) {
            for (auto& entity : buffer) {
                if (entity.type() == osmium::item_type::node) {
                    loc_handler.node(static_cast<const osmium::Node&>(entity));
                } else if (entity.type() == osmium::item_type::way) {
                    if (!indexed) { loc_handler.sort_index(); indexed = true; }
                    auto& way = static_cast<osmium::Way&>(entity);
                    loc_handler.way(way);
                    test_handler.way(way);
                }
            }
        }
        reader.close();

        // Atomarer Austausch
        {
            std::lock_guard<std::mutex> lock(cache_mutex);
            global_node_pool = std::move(thread_pool);
            map_cache = std::move(thread_cache);
        }
        std::cout << "[Thread] Update complete. Cache size: " << map_cache.size() << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "[Thread] Error: " << e.what() << std::endl;
    }
    is_updating = false;
}

// --- RENDERING ---

Point locToPixel(const osmium::Box &box, const osmium::Location &location) {
    Point p;
    p.x = ((location.lon() - box.bottom_left().lon()) / (box.top_right().lon() - box.bottom_left().lon())) * WINDOW_WIDTH;
    p.y = WINDOW_HEIGHT - (((location.lat() - box.bottom_left().lat()) / (box.top_right().lat() - box.bottom_left().lat())) * WINDOW_HEIGHT);
    return p;
}

void render(Display *display, Pixmap pixmap, GC gc, double center_lat, double center_lon, double radius_m) {
    const double earth_radius = 6371000.0;
    const double M_PI_CONST = 3.14159265358979323846;
    double lat_offset = (radius_m / earth_radius) * (180.0 / M_PI_CONST);
    double lon_offset = (radius_m / (earth_radius * std::cos(center_lat * M_PI_CONST / 180.0))) * (180.0 / M_PI_CONST);
    osmium::Box bounding_box(osmium::Location{center_lon - lon_offset, center_lat - lat_offset}, 
                             osmium::Location{center_lon + lon_offset, center_lat + lat_offset});

    std::lock_guard<std::mutex> lock(cache_mutex);
    
    for (const auto &way : map_cache) {
        if (!way.tagMap) continue;

        // Setze Farbe basierend auf Tags
        if (way.tagMap & 1) XSetForeground(display, gc, 0x000000); // Schwarz (Residential)
        else if (way.tagMap & (1 << 7)) XSetForeground(display, gc, 0x0000FF); // Blau (Water)
        else XSetForeground(display, gc, 0xCCCCCC); // Grau (Other)

        for (int i = 0; i < way.node_count - 1; i++) {
            Position p1 = global_node_pool[way.start_index + i];
            Position p2 = global_node_pool[way.start_index + i + 1];

            if (bounding_box.contains({p1.x, p1.y}) || bounding_box.contains({p2.x, p2.y})) {
                Point one = locToPixel(bounding_box, {p1.x, p1.y});
                Point two = locToPixel(bounding_box, {p2.x, p2.y});
                XDrawLine(display, pixmap, gc, one.x, one.y, two.x, two.y);
            }
        }
    }

    // Kleiner Indikator oben links wenn der Thread arbeitet
    if (is_updating) {
        XSetForeground(display, gc, 0xFF0000);
        XFillArc(display, pixmap, gc, 10, 10, 10, 10, 0, 360 * 64);
    }
}

int main() {
    Display *display = XOpenDisplay(0);
    if (!display) return -1;

    Window window = XCreateSimpleWindow(display, DefaultRootWindow(display), 0, 0, WINDOW_WIDTH, WINDOW_HEIGHT, 0,
                                        BlackPixel(display, DefaultScreen(display)), WhitePixel(display, DefaultScreen(display)));
    XSelectInput(display, window, ExposureMask | KeyPressMask);
    XMapWindow(display, window);
    GC gc = XCreateGC(display, window, 0, NULL);
    Pixmap pixmap = XCreatePixmap(display, window, WINDOW_WIDTH, WINDOW_HEIGHT, DefaultDepth(display, DefaultScreen(display)));

    Position p{54.3603481, 10.2850605};
    Position previous = p;
    double cache_radius = 2000;
    double render_radius = 500;

    // Initiales Laden (noch im Hauptthread für den ersten Frame)
    backgroundUpdateTask(p, cache_radius);

    XEvent event;
    while (1) {
        // Wir nutzen XPending um die Loop nicht zu blockieren, falls kein Event da ist
        while (XPending(display)) {
            XNextEvent(display, &event);
            if (event.type == KeyPress) {
                KeySym keysym = XLookupKeysym(&event.xkey, 0);
                if (keysym == XK_w) p.x += 0.0002;
                else if (keysym == XK_s) p.x -= 0.0002;
                else if (keysym == XK_a) p.y -= 0.0002;
                else if (keysym == XK_d) p.y += 0.0002;

                // Prüfe ob Distanz zum letzten Cache-Update > 1000m
                double dist = osmium::geom::haversine::distance(osmium::geom::Coordinates(previous.y, previous.x), 
                                                                osmium::geom::Coordinates(p.y, p.x));
                if (dist > 1000 && !is_updating) {
                    previous = p;
                    std::thread(backgroundUpdateTask, p, cache_radius).detach();
                }
            }
        }

        // Zeichnen (auf Pixmap, dann CopyArea)
        XSetForeground(display, gc, WhitePixel(display, DefaultScreen(display)));
        XFillRectangle(display, pixmap, gc, 0, 0, WINDOW_WIDTH, WINDOW_HEIGHT);
        render(display, pixmap, gc, p.x, p.y, render_radius);
        XCopyArea(display, pixmap, window, gc, 0, 0, WINDOW_WIDTH, WINDOW_HEIGHT, 0, 0);
        XFlush(display);

        usleep(16000); // ca. 60 FPS
    }

    XFreeGC(display, gc);
    XCloseDisplay(display);
    return 0;
}