#include <exception>
#include <iostream> // for std::cout, std::cerr

// Allow any format of input files (XML, PBF, ...)
#include <osmium/io/any_input.hpp>

// For the osmium::geom::haversine::distance() function
#include <osmium/geom/haversine.hpp>

// For osmium::apply()
#include <osmium/visitor.hpp>

// For the location index. There are different types of indexes available.
// This will work for all input files keeping the index in memory.
#include <osmium/index/map/flex_mem.hpp>

// For the NodeLocationForWays handler
#include <osmium/handler/node_locations_for_ways.hpp>

#include <osmium/osm/location.hpp>
#include <osmium/geom/mercator_projection.hpp>
#include <osmium/geom/tile.hpp>

// The type of index used. This must match the include file above
using index_type = osmium::index::map::FlexMem<osmium::unsigned_object_id_type, osmium::Location>;

// The location handler always depends on the index type
using location_handler_type = osmium::handler::NodeLocationsForWays<index_type>;

#include <X11/Xlib.h>
#include <X11/Xutil.h>
#include <unistd.h>

#define WINDOW_HEIGHT 800
#define WINDOW_WIDTH 800

struct Point
{
    int x;
    int y;
};

struct CachedWay
{
    osmium::WayNodeList wnl;
    osmium::TagList tags;
};

std::vector<CachedWay> map_cache;

struct TestHandler : public osmium::handler::Handler
{
    std::vector<CachedWay> map_cache;
    Display *display;
    Pixmap pixmap;
    GC gc;
    osmium::Box bounding_box; // Speichern der berechneten Box

    // Konstruktor mit Radius-Logik
    TestHandler(std::vector<CachedWay> *map_cache, Display **display, Pixmap *pixmap, GC *gc, double center_lat, double center_lon, double radius_m)
    {
        this->map_cache = *map_cache;
        this->display = *display;
        this->pixmap = *pixmap;
        this->gc = *gc;

        // Erdradius für die Berechnung
        const double earth_radius = 6371000.0;
        const double M_PI_CONST = 3.14159265358979323846;

        // Offsets berechnen
        double lat_offset = (radius_m / earth_radius) * (180.0 / M_PI_CONST);
        double lon_offset = (radius_m / (earth_radius * std::cos(center_lat * M_PI_CONST / 180.0))) * (180.0 / M_PI_CONST);

        // Box erstellen (Süd-West bis Nord-Ost)
        osmium::Location sw{center_lon - lon_offset, center_lat - lat_offset};
        osmium::Location ne{center_lon + lon_offset, center_lat + lat_offset};
        this->bounding_box = osmium::Box(sw, ne);
    }

    Point locToPixel(const osmium::Box &box, const osmium::Location &location)
    {
        Point p;
        // Normalisierung der Koordinaten auf die Fenstergröße
        p.x = ((location.lon() - box.bottom_left().lon()) / (box.top_right().lon() - box.bottom_left().lon())) * WINDOW_WIDTH;
        p.y = WINDOW_HEIGHT - (((location.lat() - box.bottom_left().lat()) / (box.top_right().lat() - box.bottom_left().lat())) * WINDOW_HEIGHT);
        return p;
    }

    void way(const osmium::Way &way)
    {
        const char *highway = way.tags()["highway"];
        if (highway)
        {
            const osmium::WayNodeList &wnl = way.nodes();
            for (auto it = wnl.begin(); it != wnl.end(); ++it)
            {
                auto next_it = std::next(it);
                if (next_it != wnl.end())
                {
                    // Prüfen, ob die Locations valide sind, bevor wir rechnen
                    if (it->location().valid() && next_it->location().valid())
                    {
                        if(bounding_box.contains(it->location())) {
                            CachedWay c{way.nodes(), way.tags()};
                            map_cache.push_back(c);
                        }

                        Point one = locToPixel(bounding_box, it->location());
                        Point two = locToPixel(bounding_box, next_it->location());

                        
                        // Einfache Logik für Linienstärke/Farbe
                        if (way.tags().has_tag("highway", "residential") || way.tags().has_tag("highway", "secondary") || way.tags().has_tag("highway", "living_street"))
                        {
                            XSetForeground(display, gc, BlackPixel(display, DefaultScreen(display)));
                        }
                        else
                        {
                            XSetForeground(display, gc, 0xCCCCCC);
                        }

                        XDrawLine(display, pixmap, gc, one.x, one.y, two.x, two.y);
                    }
                }
            }
        }
    }
};

int main()
{
    // Open a display connection to the X server
    Display *display = XOpenDisplay(0);
    if (!display)
    {
        return -1;
    }

    // Create a simple window with a size of 200x100 pixels
    Window window = XCreateSimpleWindow(
        display,
        DefaultRootWindow(display),
        0, 0, WINDOW_WIDTH, WINDOW_HEIGHT, 0,
        BlackPixel(display, DefaultScreen(display)),
        WhitePixel(display, DefaultScreen(display)));

    // Set window properties such as title
    XSetStandardProperties(display, window, "My Window", "My Window", None, NULL, 0, NULL);

    // Select the types of events the window will receive
    XSelectInput(display, window, ExposureMask | ButtonPressMask | KeyPressMask);

    // Create a graphics context for drawing
    GC gc = XCreateGC(display, window, 0, NULL);

    // Set foreground and background colors

    // Map the window to make it visible
    XMapWindow(display, window);

    // Flush the output buffer to ensure the window is displayed
    XFlush(display);

    int depth = DefaultDepth(display, DefaultScreen(display));
    Pixmap pixmap = XCreatePixmap(display, window, WINDOW_WIDTH, WINDOW_HEIGHT, depth);

    // Load the map

    // Wait for events (e.g., key press or button press)
    XEvent event;
    char buffer[32];
    KeySym keysym;

    struct Position
    {
        double x;
        double y;
    };

    Position p{54.3603481, 10.2850605};
    TestHandler handler{&display, &pixmap, &gc, p.x, p.y, 500};
    index_type index;
    location_handler_type location_handler{index};

    osmium::io::Reader reader{"./data/map.osm", osmium::osm_entity_bits::node | osmium::osm_entity_bits::way};
    osmium::apply(reader, location_handler, handler);
    reader.close();

    while (1)
    {
        XNextEvent(display, &event);
        if (event.type == Expose && event.xexpose.count == 0)
        {
            XCopyArea(display, pixmap, window, gc, 0, 0, WINDOW_WIDTH, WINDOW_HEIGHT, 0, 0);
        }
        if (event.type == KeyPress)
        {
            int len = XLookupString(&event.xkey, buffer, sizeof(buffer), &keysym, NULL);
            if (len > 0)
            {
                XSetForeground(display, gc, WhitePixel(display, DefaultScreen(display)));
                XFillRectangle(display, pixmap, gc, 0, 0, WINDOW_WIDTH, WINDOW_HEIGHT);

                if (buffer[0] == 'w')
                {
                    p.x += 0.0002;
                }
                else if (buffer[0] == 's')
                {
                    p.x -= 0.0002;
                }
                else if (buffer[0] == 'a')
                {
                    p.y -= 0.0002;
                }
                else if (buffer[0] == 'd')
                {
                    p.y += 0.0002;
                }

                XCopyArea(display, pixmap, window, gc, 0, 0, WINDOW_WIDTH, WINDOW_HEIGHT, 0, 0);
            }
        }
    }

    // Clean up resources
    XFreeGC(display, gc);
    XDestroyWindow(display, window);
    XCloseDisplay(display);

    return 0;
}