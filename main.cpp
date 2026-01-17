/*

  EXAMPLE osmium_road_length

  Calculate the length of the road network (everything tagged `highway=*`)
  from the given OSM file.

  DEMONSTRATES USE OF:
  * file input
  * location indexes and the NodeLocationsForWays handler
  * length calculation on the earth using the haversine function

  SIMPLER EXAMPLES you might want to understand first:
  * osmium_read
  * osmium_count
  * osmium_pub_names

  LICENSE
  The code in this example file is released into the Public Domain.

*/

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


struct TestHandler : public osmium::handler::Handler {
    void node(const osmium::Node& node) noexcept {
        node.location().
    }
};


int main() {
    // Open a display connection to the X server
    Display *display = XOpenDisplay(0);
    if (!display) {
        return -1;
    }

    // Create a simple window with a size of 200x100 pixels
    Window window = XCreateSimpleWindow(
        display,
        DefaultRootWindow(display),
        0, 0, 800, 800, 0,
        BlackPixel(display, DefaultScreen(display)),
        WhitePixel(display, DefaultScreen(display))
    );

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
    Pixmap pixmap = XCreatePixmap(display, window, 800, 800, depth);

    XSetForeground(display, gc, WhitePixel(display, DefaultScreen(display)));
    XFillRectangle(display, pixmap, gc, 0, 0, 800, 800);

    XSetForeground(display, gc, BlackPixel(display, DefaultScreen(display)));
    XDrawLine(display, pixmap, gc, 20, 400, 650, 530);


    // Load the map
    osmium::io::Reader reader{"./data/map.osm", osmium::osm_entity_bits::node | osmium::osm_entity_bits::way};

    osmium::Location location{};
    try{
        location.set_lon("10.2850605");
        location.set_lat("54.3603481");
    }
    catch (const osmium::invalid_location&) {
        std::cerr << "ERROR: Location is invalid\n";
        return 1;
    }

    const osmium::geom::Coordinates c = osmium::geom::lonlat_to_mercator(location);

    const uint32_t zoom = 16;
    const osmium::geom::Tile tile{static_cast<uint32_t>(zoom), location};

    std::cout << "X: " << tile.x << "Y: " << tile.y << "\n";

    // Wait for events (e.g., key press or button press)
    XEvent event;
    while (1) {
        XNextEvent(display, &event);
        if(event.type == Expose && event.xexpose.count == 0) {
            XCopyArea(display, pixmap, window, gc, 0, 0, 800, 800, 0, 0);
        }
        if (event.type == KeyPress) {
            XCopyArea(display, pixmap, window, gc, 0, 0, 800, 800, 0, 0);
        }
    }

    // Clean up resources
    XFreeGC(display, gc);
    XDestroyWindow(display, window);
    XCloseDisplay(display);

    return 0;
}   