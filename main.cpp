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
#include <Imlib2.h>
#include <stdio.h>

#define WINDOW_HEIGHT 500
#define WINDOW_WIDTH 500

struct Point
{
    int x, y;
};

struct Position
{
    float x, y;
};

// --- GLOBALE DATENSTRUKTUREN ---
struct CachedWay
{
    uint32_t start_index = 0;
    uint16_t node_count = 0;
    uint8_t tagMap = 0;
};

std::vector<Position> global_node_pool;
std::vector<CachedWay> map_cache;
std::mutex cache_mutex;
std::atomic<bool> is_updating{false};
std::atomic<bool> needs_redraw{true};

#define MAX_HOMES 4
Position homes[MAX_HOMES];

struct NodeEntry
{
    osmium::object_id_type id;
    osmium::Location location;
    bool operator<(const NodeEntry &other) const { return id < other.id; }
};

// Navigation

// 8 byte
// 4 + 4 byte

// 8 byte
// 4 byte
// 8 byte

struct RoutingNode
{
    osmium::object_id_type osmium_id;
    Position p;
    struct Edge
    {
        osmium::object_id_type target_id;
        float weight;
        osmium::object_id_type way_id;
    };
    std::vector<Edge> edges;
};

std::vector<RoutingNode> routingMesh;

struct RoutingMeshHandler : public osmium::handler::Handler {
    std::vector<osmium::object_id_type> node_refs;

    bool is_relevant_highway(const osmium::Way& way) {
        const char* highway = way.tags()["highway"];
        if (!highway) return false;
        static const std::vector<std::string> ignore = {
            "footway", "cycleway", "path", "service", "track", "steps", "pedestrian"
        };
        for (const auto& s : ignore) if (s == highway) return false;
        return true;
    }

    int getSpeedForWay(const osmium::Way& way) {
        const char* maxspeed_str = way.get_value_by_key("maxspeed");
        if (maxspeed_str) {
            try {
                int val = std::stoi(maxspeed_str);
                if (val > 0) return val;
            } catch (...) {
                if (std::string(maxspeed_str) == "walk") return 7;
            }
        }

        const char* maxspeedtype = way.get_value_by_key("maxspeed:type");
        if (maxspeedtype) {
            std::string type = maxspeedtype;
            if (type == "DE:rural") return 100;
            if (type == "DE:urban") return 50;
            if (type == "DE:motorway") return 130;
        }

        // Fallback basierend auf Highway-Klasse
        const char* highway = way.get_value_by_key("highway");
        if (highway) {
            std::string h = highway;
            if (h == "motorway") return 130;
            if (h == "trunk") return 100;
            if (h == "primary") return 100;
            if (h == "secondary") return 80;
            if (h == "tertiary") return 70;
            if (h == "residential") return 30;
            if (h == "living_street") return 7;
        }

        return 50; // Standard-Fallback
    }

    // PASS 1: Sammeln
    void way(const osmium::Way& way) {
        if (!is_relevant_highway(way)) return;
        const auto& nodes = way.nodes();
        if (nodes.size() < 2) return;

        for (size_t i = 0; i < nodes.size(); ++i) {
            node_refs.push_back(nodes[i].ref());
            if (i == 0 || i == nodes.size() - 1) {
                node_refs.push_back(nodes[i].ref());
            }
        }
    }

    void createMeshNodes() {
        if (node_refs.empty()) return;
        std::sort(node_refs.begin(), node_refs.end());

        for (size_t i = 0; i < node_refs.size() - 1; ++i) {
            if (node_refs[i] == node_refs[i+1]) {
                routingMesh.push_back(RoutingNode{node_refs[i], {0,0}, {}});
                osmium::object_id_type current_id = node_refs[i];
                while (i < node_refs.size() && node_refs[i] == current_id) i++;
                i--;
            }
        }
        node_refs.clear();
        node_refs.shrink_to_fit();

        std::sort(routingMesh.begin(), routingMesh.end(), [](const RoutingNode& a, const RoutingNode& b) {
            return a.osmium_id < b.osmium_id;
        });
    }

    void assignLocations(const osmium::Node& node) {
        auto it = std::lower_bound(routingMesh.begin(), routingMesh.end(), node.id(), 
            [](const RoutingNode& rn, osmium::object_id_type id) {
                return rn.osmium_id < id;
            });

        if (it != routingMesh.end() && it->osmium_id == node.id()) {
            it->p = { (float)node.location().lat(), (float)node.location().lon() };
        }
    }

    RoutingNode* findInMesh(osmium::object_id_type id) {
        auto it = std::lower_bound(routingMesh.begin(), routingMesh.end(), id,
            [](const RoutingNode& node, osmium::object_id_type id) {
                return node.osmium_id < id;
            });
        if (it != routingMesh.end() && it->osmium_id == id) {
            return &(*it);
        }
        return nullptr;
    }

    void buildEdges(const osmium::Way& way) {
        if (!is_relevant_highway(way)) return;

        const auto& nodes = way.nodes();
        if (nodes.size() < 2) return;

        int speed = getSpeedForWay(way);
        RoutingNode* lastMeshNode = nullptr;
        double accumulatedDistance = 0.0;

        for (size_t i = 0; i < nodes.size(); ++i) {
            RoutingNode* currentMeshNode = findInMesh(nodes[i].ref());

            if (currentMeshNode) {
                if (lastMeshNode) {
                    // Haversine distance between the two mesh nodes (ignoring curves)
                    osmium::Location loc1(lastMeshNode->p.y, lastMeshNode->p.x);
                    osmium::Location loc2(currentMeshNode->p.y, currentMeshNode->p.x);
                    
                    if (loc1.valid() && loc2.valid()) {
                        double dist = osmium::geom::haversine::distance(loc1, loc2);
                        float weight = (float)(dist / (speed / 3.6)); // Time in seconds
                        
                        lastMeshNode->edges.push_back({currentMeshNode->osmium_id, weight, way.id()});
                        
                        const char* ow = way.get_value_by_key("oneway");
                        if (!(ow && std::string(ow) == "yes")) {
                            currentMeshNode->edges.push_back({lastMeshNode->osmium_id, weight, way.id()});
                        }

                        if(currentMeshNode->osmium_id == 43762102 || lastMeshNode->osmium_id == 437621029999999999){
                            std::cout << "parsed your intersection\n";
                            std::cout << currentMeshNode->osmium_id << "; " << currentMeshNode->edges.size()<<";\n";
                           // std::cout << lastMeshNode->osmium_id << "; " << lastMeshNode->edges.size()<<";\n";

                           for(const auto &edge : currentMeshNode->edges) {
                                std::cout << edge.target_id << "; " << edge.way_id << "; " << edge.weight << ";\n";
                           }
                            std::cout << "\n";

                            
                        }
                    }
                }
                lastMeshNode = currentMeshNode;
            }
        }
    }
};

// --- HANDLER ---

struct MySmartLocationHandler : public osmium::handler::Handler
{
    std::vector<NodeEntry> local_index;
    osmium::Box &filter_box;

    MySmartLocationHandler(osmium::Box &box) : filter_box(box) {}

    void node(const osmium::Node &node)
    {
        if (filter_box.contains(node.location()))
        {
            local_index.push_back({node.id(), node.location()});
        }
    }

    void sort_index()
    {
        std::sort(local_index.begin(), local_index.end());
    }

    void way(osmium::Way &way)
    {
        for (auto &node_ref : way.nodes())
        {
            auto it = std::lower_bound(local_index.begin(), local_index.end(), NodeEntry{node_ref.ref(), {0, 0}});
            if (it != local_index.end() && it->id == node_ref.ref())
            {
                node_ref.set_location(it->location);
            }
        }
    }
};

struct TestHandler : public osmium::handler::Handler
{
    std::vector<CachedWay> &local_cache;
    std::vector<Position> &local_pool;
    osmium::Box bounding_box;

    TestHandler(std::vector<CachedWay> &cache_out, std::vector<Position> &pool_out, double center_lat, double center_lon, double radius_m)
        : local_cache(cache_out), local_pool(pool_out)
    {
        const double earth_radius = 6371000.0;
        const double M_PI_CONST = 3.14159265358979323846;
        double lat_offset = (radius_m / earth_radius) * (180.0 / M_PI_CONST);
        double lon_offset = (radius_m / (earth_radius * std::cos(center_lat * M_PI_CONST / 180.0))) * (180.0 / M_PI_CONST);
        this->bounding_box = osmium::Box(osmium::Location{center_lon - lon_offset, center_lat - lat_offset},
                                         osmium::Location{center_lon + lon_offset, center_lat + lat_offset});
    }

    void way(const osmium::Way &way)
    {
        const char *highway = way.tags()["highway"];
        const char *waterway = way.tags()["waterway"];
        const char *building = way.tags()["building"];
        if (way.tags().has_tag("highway", "footway") || way.tags().has_tag("highway", "path"))
        {
            return;
        }
        if (highway || waterway || building)
        {
            bool any_node_in_box = false;
            for (const auto &node : way.nodes())
            {
                if (node.location().valid() && bounding_box.contains(node.location()))
                {
                    any_node_in_box = true;
                    break;
                }
            }

            if (any_node_in_box)
            {
                CachedWay c;
                c.start_index = static_cast<uint32_t>(local_pool.size());
                for (const auto &node : way.nodes())
                {
                    if (node.location().valid())
                    {
                        local_pool.push_back({static_cast<float>(node.lon()), static_cast<float>(node.lat())});
                        c.node_count++;
                    }
                }
                if (way.tags().has_tag("highway", "residential") || way.tags().has_tag("highway", "secondary") ||
                    way.tags().has_tag("highway", "tertiary") || way.tags().has_tag("highway", "living_street"))
                {
                    c.tagMap |= 1;
                }
                else if (highway)
                {
                    c.tagMap |= 1 << 1;
                }
                else if (building)
                {
                    c.tagMap |= 1 << 4;
                }
                else if (way.tags().has_tag("waterway", "river"))
                {
                    c.tagMap |= 1 << 7;
                }
                local_cache.push_back(std::move(c));
            }
        }
    }
};

// --- THREAD LOGIK ---

void backgroundUpdateTask(Position p, double radius)
{
    is_updating = true;
    std::vector<Position> thread_pool;
    std::vector<CachedWay> thread_cache;

    const double earth_radius = 6371000.0;
    const double M_PI_CONST = 3.14159265358979323846;
    double lat_offset = (radius / earth_radius) * (180.0 / M_PI_CONST);
    double lon_offset = (radius / (earth_radius * std::cos(p.x * M_PI_CONST / 180.0))) * (180.0 / M_PI_CONST);
    osmium::Box thread_box(osmium::Location{p.y - lon_offset, p.x - lat_offset},
                           osmium::Location{p.y + lon_offset, p.x + lat_offset});

    MySmartLocationHandler loc_handler{thread_box};
    TestHandler test_handler{thread_cache, thread_pool, p.x, p.y, radius};

    try
    {
        osmium::io::Reader reader{"./data/sh-map.pbf", osmium::osm_entity_bits::node | osmium::osm_entity_bits::way};
        bool indexed = false;
        while (osmium::memory::Buffer buffer = reader.read())
        {
            for (auto &entity : buffer)
            {
                if (entity.type() == osmium::item_type::node)
                {
                    loc_handler.node(static_cast<const osmium::Node &>(entity));
                }
                else if (entity.type() == osmium::item_type::way)
                {
                    if (!indexed)
                    {
                        loc_handler.sort_index();
                        indexed = true;
                    }
                    auto &way = static_cast<osmium::Way &>(entity);
                    loc_handler.way(way);
                    test_handler.way(way);
                }
            }
        }
        reader.close();

        {
            std::lock_guard<std::mutex> lock(cache_mutex);
            global_node_pool = std::move(thread_pool);
            map_cache = std::move(thread_cache);
        }
        needs_redraw = true;
    }
    catch (...)
    {
    }
    is_updating = false;
    needs_redraw = true;
}

void loadRoutingMesh()
{
    try
    {
        osmium::io::Reader reader1{"./data/sh-map.pbf", osmium::osm_entity_bits::way};

        RoutingMeshHandler routingMeshHandler;

        while (osmium::memory::Buffer buffer = reader1.read())
        {
            for (auto &entity : buffer)
            {
                if (entity.type() == osmium::item_type::way)
                {
                    routingMeshHandler.way((osmium::Way &)entity);
                }
            }
        }
        reader1.close();
        routingMeshHandler.createMeshNodes();

        osmium::io::Reader reader2{"./data/sh-map.pbf", osmium::osm_entity_bits::node | osmium::osm_entity_bits::way};

        while (osmium::memory::Buffer buffer = reader2.read())
        {
            for (auto &entity : buffer)
            {
                if (entity.type() == osmium::item_type::node)
                {
                    routingMeshHandler.assignLocations((osmium::Node &)entity);
                } else if(entity.type() == osmium::item_type::way) {
                    routingMeshHandler.buildEdges((osmium::Way &) entity);
                }
            }
        }
        reader2.close();
    }
    catch (...)
    {
    }
}

// --- RENDERING ---

Point locToPixel(const osmium::Box &box, const osmium::Location &location)
{
    Point p;
    p.x = ((location.lon() - box.bottom_left().lon()) / (box.top_right().lon() - box.bottom_left().lon())) * WINDOW_WIDTH;
    p.y = WINDOW_HEIGHT - (((location.lat() - box.bottom_left().lat()) / (box.top_right().lat() - box.bottom_left().lat())) * WINDOW_HEIGHT);
    return p;
}

void render_map(Display *display, Pixmap pixmap, GC gc, double center_lat, double center_lon, double radius_m)
{
    const double earth_radius = 6371000.0;
    const double M_PI_CONST = 3.14159265358979323846;
    double lat_offset = (radius_m / earth_radius) * (180.0 / M_PI_CONST);
    double lon_offset = (radius_m / (earth_radius * std::cos(center_lat * M_PI_CONST / 180.0))) * (180.0 / M_PI_CONST);
    osmium::Box bounding_box(osmium::Location{center_lon - lon_offset, center_lat - lat_offset},
                             osmium::Location{center_lon + lon_offset, center_lat + lat_offset});

    std::lock_guard<std::mutex> lock(cache_mutex);

    for (const auto &way : map_cache)
    {
        if (!way.tagMap)
            continue;

        if (way.tagMap & (1 << 4))
        {
            std::vector<XPoint> points(way.node_count);
            for (int i = 0; i < way.node_count; i++)
            {
                Point p = locToPixel(bounding_box, {global_node_pool[way.start_index + i].x, global_node_pool[way.start_index + i].y});
                points[i] = {(short)(p.x), (short)(p.y)};
            }
            XSetFillStyle(display, gc, FillSolid);
            XSetForeground(display, gc, 0x969696);
            XFillPolygon(display, pixmap, gc, points.data(), way.node_count, Nonconvex, CoordModeOrigin);
            continue;
        }

        if (way.tagMap & 1)
        {
            XSetForeground(display, gc, 0xFFFFFF);
            XSetLineAttributes(display, gc, 7, 0, 0, 0);
        }
        else if (way.tagMap & (1 << 7))
        {
            XSetForeground(display, gc, 0x0000FF);
        }
        else
        {
            XSetForeground(display, gc, 0xD16E5B);
            XSetLineAttributes(display, gc, 4, 0, 0, 0);
        }
        XPoint points[way.node_count];
        for (int i = 0; i < way.node_count; i++)
        {
            Point p = locToPixel(bounding_box, {global_node_pool[way.start_index + i].x, global_node_pool[way.start_index + i].y});
            points[i] = {(short)(p.x), (short)(p.y)};
        }
        XDrawLines(display, pixmap, gc, points, way.node_count, CoordModeOrigin);

        XSetLineAttributes(display, gc, 1, 0, 0, 0);
    }

    if (is_updating)
    {
        XSetForeground(display, gc, 0x2ECC71);
        XFillArc(display, pixmap, gc, WINDOW_WIDTH - 30, 20, 12, 12, 0, 360 * 64);
    }
}
// Neue Funktion zum Zeichnen mit Alpha-Blending via Imlib2
void draw_image_alpha(Display *dpy, Pixmap target, Imlib_Image src_img, int x, int y, int scaleX, int scaleY)
{
    if (!src_img)
        return;

    imlib_context_set_image(src_img);

    imlib_context_set_display(dpy);
    imlib_context_set_visual(DefaultVisual(dpy, DefaultScreen(dpy)));
    imlib_context_set_colormap(DefaultColormap(dpy, DefaultScreen(dpy)));
    imlib_context_set_drawable(target);

    // Blend-Modus aktivieren (das ist der entscheidende Teil für Transparenz)
    imlib_context_set_blend(1);

    // Rendert das Bild an Position (x, y) auf die Pixmap
    // Zentriert: (WINDOW_WIDTH - w)/2
    imlib_render_image_on_drawable_at_size(x, y, scaleX, scaleY);
}

void render_homes(Display *display, Pixmap pixmap, Imlib_Image houseImage, Position currentPosition, double radius_m)
{
    const double earth_radius = 6371000.0;
    const double M_PI_CONST = 3.14159265358979323846;
    double lat_offset = (radius_m / earth_radius) * (180.0 / M_PI_CONST);
    double lon_offset = (radius_m / (earth_radius * std::cos(currentPosition.x * M_PI_CONST / 180.0))) * (180.0 / M_PI_CONST);
    osmium::Box bounding_box(osmium::Location{currentPosition.y - lon_offset, currentPosition.x - lat_offset},
                             osmium::Location{currentPosition.y + lon_offset, currentPosition.x + lat_offset});

    for (const Position house : homes)
    {
        if (house.x == -190)
        {
            continue;
        }
        Point p = locToPixel(bounding_box, osmium::Location{house.y, house.x});
        draw_image_alpha(display, pixmap, houseImage, p.x - 64, p.y - 128, 128, 128);
    }
}

void saveNewHome(Position *p)
{
    bool changed = false;
    for (int i = 0; i < MAX_HOMES; i++)
    {
        // -190 is no valid coordinate so this is a non-existant coordinate
        if (homes[i].x == -190)
        {
            homes[i] = Position{*p};
            changed = true;
            break;
        }
    }

    if (!changed)
    {
        //std::cout << "Max Houses already reached.";
    }
}

int main()
{
    Display *display = XOpenDisplay(0);
    if (!display)
        return -1;

    int screen = DefaultScreen(display);
    Window window = XCreateSimpleWindow(display, DefaultRootWindow(display), 0, 0, WINDOW_WIDTH, WINDOW_HEIGHT, 0,
                                        BlackPixel(display, screen), WhitePixel(display, screen));

    XSelectInput(display, window, ExposureMask | KeyPressMask);
    XMapWindow(display, window);

    GC gc = XCreateGC(display, window, 0, NULL);
    Pixmap pixmap = XCreatePixmap(display, window, WINDOW_WIDTH, WINDOW_HEIGHT, DefaultDepth(display, screen));

    // Wir laden das Bild als Imlib_Image und behalten es so (für Alpha Support)
    Imlib_Image playerArrowImg = imlib_load_image("./resources/fh4_player_arrow.png");
    if (!playerArrowImg)
    {
        fprintf(stderr, "Fehler: Bild konnte nicht geladen werden.\n");
        return -1;
    }

    int arrowWidth = 42;
    int arrowHeight = 42;

    Imlib_Image vingetteImg = imlib_load_image("./resources/vingette.png");

    Imlib_Image houseImg = imlib_load_image("./resources/house.png");

    Position p{54.3603481, 10.2850605};
    Position previous = p;
    double cache_radius = 2000;
    double render_radius = 150;

    for (int i = 0; i < MAX_HOMES; i++)
    {
        homes[i] = Position{-190, 0};
    }

    // Erster Fetch
    backgroundUpdateTask(p, cache_radius);

    loadRoutingMesh();

    XEvent event;
    while (1)
    {
        if (!XPending(display) && !needs_redraw)
        {
            usleep(10000);
            continue;
        }

        while (XPending(display))
        {
            XNextEvent(display, &event);
            if (event.type == Expose)
            {
                needs_redraw = true;
            }
            if (event.type == KeyPress)
            {
                KeySym keysym = XLookupKeysym(&event.xkey, 0);
                float step = 0.0002;
                if (keysym == XK_w)
                    p.x += step;
                else if (keysym == XK_s)
                    p.x -= step;
                else if (keysym == XK_a)
                    p.y -= step;
                else if (keysym == XK_d)
                    p.y += step;
                else if (keysym == XK_e)
                {
                    saveNewHome(&p);
                }
                else if (keysym == XK_Escape)
                    goto cleanup;

                needs_redraw = true;

                double dist = osmium::geom::haversine::distance(osmium::geom::Coordinates(previous.y, previous.x),
                                                                osmium::geom::Coordinates(p.y, p.x));
                if (dist > 1000 && !is_updating)
                {
                    previous = p;
                    std::thread(backgroundUpdateTask, p, cache_radius).detach();
                }
            }
        }

        if (needs_redraw)
        {
            // 1. Hintergrund löschen
            XSetForeground(display, gc, 0x0A0A11);
            XFillRectangle(display, pixmap, gc, 0, 0, WINDOW_WIDTH, WINDOW_HEIGHT);

            // 2. Map rendern
            render_map(display, pixmap, gc, p.x, p.y, render_radius);

            // 3. Player Arrow mit Transparenz drüberblenden
            draw_image_alpha(display, pixmap, playerArrowImg,
                             (WINDOW_WIDTH - arrowWidth) / 2,
                             (WINDOW_HEIGHT - arrowHeight) / 2,
                             arrowWidth, arrowHeight);

            render_homes(display, pixmap, houseImg, p, render_radius);

            draw_image_alpha(display, pixmap, vingetteImg,
                             0, 0, WINDOW_WIDTH, WINDOW_HEIGHT);

            XSetForeground(display, gc, 0x969696);
            XSetLineAttributes(display, gc, 2, 0, 0, 0);
            XDrawArc(display, pixmap, gc, 0, 0, WINDOW_WIDTH - 2, WINDOW_HEIGHT - 2, 120 * 64, 300 * 64);

            // 4. Backbuffer auf Window kopieren
            XCopyArea(display, pixmap, window, gc, 0, 0, WINDOW_WIDTH, WINDOW_HEIGHT, 0, 0);
            XFlush(display);
            needs_redraw = false;
        }
    }

cleanup:
    imlib_context_set_image(playerArrowImg);
    imlib_free_image();
    imlib_context_set_image(vingetteImg);
    imlib_free_image();
    imlib_context_set_image(houseImg);
    imlib_free_image();
    XFreeGC(display, gc);
    XFreePixmap(display, pixmap);
    XCloseDisplay(display);
    return 0;
}

/*
NOTES FOR THE NAVIGATION SYSTEM:

1. Load only all of the intersections of the map
    - only get "highway", but ignore footpaths etc.
    - possible object for each Node:
        - DONE: ID (taken from OSM)
        - Position
        - Vector of all connected Nodes with weights (distance + driving speed + ?) and corresponding original Way ID
    - all nodes get saved in an ordered array for easy lookup via e.g. binary search

2. When Navigation should start: Run A* from current position to target (get nearest node for each via fancy math) (new thread)
    - get all the Edges that where used -> get all the ways, load the corresponding OSM-Nodes and render them
    - all nodes should be ordered from start to finish for later use

3. While driving there:
    - once a second: get the nearest OSM Node on path, render remaining path purple, rest normal, because we've already been there
    - once a second: check whether we are still on the path:
        - get the nearest OSM Node on path, if threshold is too big, restart at 2

4. Once there: If the last node is the closest node -> finish navigation

*/