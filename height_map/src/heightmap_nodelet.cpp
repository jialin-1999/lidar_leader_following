#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include <height_map/heightmap.h>

namespace height_map
{
    class HeightMapNodelet: public nodelet::Nodelet
    {
    public:
        HeightMapNodelet() {}
        ~HeightMapNodelet() {}

      void onInit(void)
      {
          heightmap_.reset(new HeightMap(getNodeHandle(), getPrivateNodeHandle()));
      }

    private:
        boost::shared_ptr<HeightMap> heightmap_;
    };
};
// namespace height_map

// Register this plugin with pluginlib.  Names must match height_map_nodelet.xml.
// parameters: package, class name, class type, base class type
PLUGINLIB_DECLARE_CLASS(height_map, HeightMapNodelet, height_map::HeightMapNodelet, nodelet::Nodelet);
