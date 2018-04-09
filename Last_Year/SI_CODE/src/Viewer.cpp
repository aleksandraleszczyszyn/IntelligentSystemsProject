
#include <Viewer.h>

Viewer::Viewer()
{}

Viewer::~Viewer()
{}

void Viewer::VisualizeSideBySide(const PointCloud<PointT>::Ptr leftWindowPC,
                        const PointCloud<PointT>::Ptr rightWindowPC) const
{
    PCLVisualizer viz;

    // window size
    viz.setSize(600, 300);

    // Right View
    int v1(0);
    viz.createViewPort(0.0, 0.0, 0.5, 1.0, v1);
    viz.setBackgroundColor (0.5, 0.5, 0.5, v1);
    viz.addText("Right View ", 10, 10, "v1 text", v1);

    viz.addPointCloud(leftWindowPC, "L", v1);

    // Left View
    int v2(1);
    viz.createViewPort(0.5, 0.0, 1.0, 1.0, v2);
    viz.setBackgroundColor (0.5, 0.5, 0.5, v2);
    viz.addText("Left View ", 10, 10, "v2 text", v2);

    viz.addPointCloud (rightWindowPC, "R", v2);

    viz.resetCamera();

    // Give control over to the visualizer
    viz.spin ();
}

void Viewer::VisualizeKeypointsInPointCloud(const PointCloud<PointT>::Ptr pc,
                          const PointCloud<PointT>::Ptr keypoints) const
{
    PCLVisualizer viz;
    viz.addPointCloud(pc, "pc");

    // Draw each keypoint as a sphere
    for (size_t i = 0; i < keypoints->size(); i++)
    {
        // Get the point data
        const PointT &point = keypoints->points[i];

        // Generate a unique id for each sphere
        stringstream ss ("keypoint");
        ss << i;

        // Add a sphere at the keypoint position with radius 0.005f and color: 0.25, 0.25, 1.0 (RGB)
        viz.addSphere (point, 0.005f, 0.25, 0.25, 1.0, ss.str ());
    }

    viz.spin ();
}

void Viewer::ViewSpinImageHistogram(const SpinImage &descriptor) const
{
    PCLPlotter plotter;

    plotter.setColorScheme(vtkColorSeries::WILD_FLOWER);

    string id = "keypoint";
    std::vector<double> array_x(descriptor.descriptorSize()), array_y(descriptor.descriptorSize());
  
    // Parse the cloud data and store it in an array
    for (int j = 0; j < descriptor.descriptorSize(); ++j)
    {
    	array_x[j] = j;
    	array_y[j] = descriptor.histogram[j];
    }
  
    plotter.addPlotData(array_x, array_y, id.c_str(), vtkChart::POINTS);

    // print the spin image to the console
    cout << descriptor << endl;

    plotter.plot();
}
