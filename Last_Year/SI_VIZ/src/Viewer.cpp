

#include <Viewer.h>

Viewer::Viewer()
{
	//TODO
}

Viewer::~Viewer()
{
	//TODO
}

void Viewer::VisualizeSideBySide(const PointCloud<PointT>::Ptr leftWindowPC,
                        const PointCloud<PointT>::Ptr rightWindowPC) const
{
    PCLVisualizer viz;

    viz.setSize(600, 300);

    int v1(0);
    viz.createViewPort(0.0, 0.0, 0.5, 1.0, v1);
    viz.setBackgroundColor (0.5, 0.5, 0.5, v1);
    viz.addText("Original PC ", 10, 10, "v1 text", v1);

    viz.addPointCloud(leftWindowPC, "L", v1);

    int v2(1);
    viz.createViewPort(0.5, 0.0, 1.0, 1.0, v2);
    viz.setBackgroundColor (0.5, 0.5, 0.5, v2);
    viz.addText("Keypoints ", 10, 10, "v2 text", v2);

    viz.addPointCloud (rightWindowPC, "R", v2);

    //viz.resetCameraViewpoint("L");
    viz.resetCamera();

    // Give control over to the visualizer
    viz.spin ();
}

void Viewer::VisualizeKeypointsInPointCloud(const PointCloud<PointT>::Ptr pc,
                          const PointCloud<PointT>::Ptr keypoints) const
{
    PCLVisualizer viz;
    /*
    viz.setBackgroundColor (0.5, 0.5, 0.5);
    viz.addText("Keypoints In the Original PC ", 10, 20, "v text");
    */
    viz.setSize(500, 500);

    int v1(0);
    viz.createViewPort(0.0, 0.0, 1.0, 1.0, v1);
    viz.setBackgroundColor (0.5, 0.5, 0.5, v1);
    viz.addText("Keypoints In the Original PC ", 10, 10, "v1 text", v1);

    viz.addPointCloud(pc, "pc");

    // Draw each keypoint as a sphere
    for (size_t i = 0; i < keypoints->size(); i++)
    {
        // Get the point data
        const PointT &point = keypoints->points[i];

        // Generate a unique id for each sphere
        stringstream ss ("keypoint");
        ss << i;

        // Add a sphere at the keypoint position
        viz.addSphere (point, 0.002f, 0.25, 0.5, 0.0, ss.str());
    }

    viz.resetCamera();

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
