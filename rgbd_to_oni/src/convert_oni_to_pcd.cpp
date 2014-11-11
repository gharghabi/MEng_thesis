#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/oni_grabber.h>
#include <pcl/io/pcd_io.h>
#include <vector>
using namespace std;
using namespace pcl;
using namespace pcl::console;
typedef PointCloud<PointXYZRGBA> Cloud;
typedef Cloud::ConstPtr CloudConstPtr;
int i = 0;
char buf[4096];
//////////////////////////////////////////////////////////////////////////////
void
printHelp (int, char **argv)
{
    print_error ("Syntax is: %s input.oni\n", argv[0]);
}
//////////////////////////////////////////////////////////////////////////////
void
cloud_cb (const CloudConstPtr& cloud)
{
    PCDWriter w;
    sprintf (buf, "frame_%06d.pcd", i);
    w.writeBinaryCompressed (buf, *cloud);
    PCL_INFO ("Wrote a cloud with %lu (%ux%u) points in %s.\n",
              cloud->size (), cloud->width, cloud->height, buf);
    ++i;
}
/* ---[ */
int
main (int argc, char **argv)
{
    print_info ("Convert an ONI file to PCD format. For more information, use: %s -h\n", argv[0]);
    if (argc < 2)
    {
        printHelp (argc, argv);
        return (-1);
    }
    pcl::ONIGrabber* grabber = new pcl::ONIGrabber (argv[1], false, false);
    boost::function<void (const CloudConstPtr&) > f = boost::bind (&cloud_cb, _1);
    boost::signals2::connection c = grabber->registerCallback (f);
    while (grabber->hasDataLeft ())
    {
        grabber->start ();
    }
    PCL_INFO ("Successfully processed %d frames.\n", i);
    delete grabber;
    return (0);
}
