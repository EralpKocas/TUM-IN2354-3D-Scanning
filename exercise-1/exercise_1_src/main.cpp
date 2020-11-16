#include <iostream>
#include <fstream>
#include <array>

#include "Eigen.h"

#include "VirtualSensor.h"


struct Vertex
{
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	// position stored as 4 floats (4th component is supposed to be 1.0)
	Vector4f position;

	// color stored as 4 unsigned char
	Vector4uc color;
};

/*bool isDistValid(Vertex p1, Vertex p2, float edgeThreshold){
    float x1 = p1.position.x();
    float y1 = p1.position.y();
    float z1 = p1.position.z();

    float x2 = p2.position.x();
    float y2 = p2.position.y();
    float z2 = p2.position.z();

    float dist = pow((pow((x1-x2), 2)+ pow((y1-y2), 2) + pow((z1-z2), 2)), (0.5));

    if (dist < edgeThreshold) return TRUE;
    return FALSE;
}*/

bool isDistValid(Vertex p1, Vertex p2, Vertex p3, float edgeThreshold){
    float x1 = p1.position.x();
    float y1 = p1.position.y();
    float z1 = p1.position.z();

    float x2 = p2.position.x();
    float y2 = p2.position.y();
    float z2 = p2.position.z();

    float x3 = p3.position.x();
    float y3 = p3.position.y();
    float z3 = p3.position.z();

    float dist1 = pow((pow((x1-x2), 2)+ pow((y1-y2), 2) + pow((z1-z2), 2)), (0.5));
    float dist2 = pow((pow((x1-x3), 2)+ pow((y1-y3), 2) + pow((z1-z3), 2)), (0.5));
    float dist3 = pow((pow((x2-x3), 2)+ pow((y2-y3), 2) + pow((z2-z3), 2)), (0.5));

    if (dist1 < edgeThreshold and dist2 < edgeThreshold and dist3 < edgeThreshold) return TRUE;
    return FALSE;
}

bool WriteMesh(Vertex* vertices, unsigned int width, unsigned int height, const std::string& filename)
{
	float edgeThreshold = 0.01f; // 1cm

	// TODO 2: use the OFF file format to save the vertices grid (http://www.geomview.org/docs/html/OFF.html)
	// - have a look at the "off_sample.off" file to see how to store the vertices and triangles
	// - for debugging we recommend to first only write out the vertices (set the number of faces to zero)
	// - for simplicity write every vertex to file, even if it is not valid (position.x() == MINF) (note that all vertices in the off file have to be valid, thus, if a point is not valid write out a dummy point like (0,0,0))
	// - use a simple triangulation exploiting the grid structure (neighboring vertices build a triangle, two triangles per grid cell)
	// - you can use an arbitrary triangulation of the cells, but make sure that the triangles are consistently oriented
	// - only write triangles with valid vertices and an edge length smaller then edgeThreshold

	// TODO: Get number of vertices
    unsigned int nVertices = width * height;

	// TODO: Determine number of valid faces
	unsigned nFaces = 0;

    for(int i=0; i < nVertices; i++){
        if (i != nVertices - 1 and vertices[i].position.x() != MINF){
            if (i % 639 == 0 or i >= nVertices - width) continue;

            int corner_2 = i + width;
            int corner_3 = i + 1;
            //bool distTrue = isDistValid(vertices[corner_2], vertices[corner_3], edgeThreshold);
            bool distTrue = isDistValid(vertices[i], vertices[corner_2], vertices[corner_3], edgeThreshold);
            if (vertices[corner_2].position.x() != MINF and vertices[corner_3].position.x() != MINF and distTrue){
                nFaces++;
            } else continue;

            int corner_4 = corner_2 + 1;
            //added later, not used in first draft distance
            distTrue = isDistValid(vertices[corner_2], vertices[corner_3], vertices[corner_4], edgeThreshold);
            if (vertices[corner_4].position.x() != MINF and distTrue)
                nFaces++;
        }
    }
	// Write off file
	std::ofstream outFile(filename);
	if (!outFile.is_open()) return false;

	// write header
	outFile << "COFF" << std::endl;
	outFile << nVertices << " " << nFaces << " 0" << std::endl;

	// TODO: save vertices

	// TODO: save valid faces

    for(int i=0; i < nVertices; i++){
        Vertex *temp_vertices = new Vertex();
        if(vertices[i].position.x() == MINF){
            temp_vertices->position = Vector4f(0, 0, 0, 1);
            temp_vertices->color = vertices[i].color;

            outFile << temp_vertices->position.x() << " " << temp_vertices->position.y() << " " << temp_vertices->position.z()
            << " " << +temp_vertices->color[0] << " " << +temp_vertices->color[1]<< " " << +temp_vertices->color[2]
            << " " << +temp_vertices->color[3] << std::endl;
        }
        else{
            outFile << vertices[i].position.x() << " " << vertices[i].position.y() << " " << vertices[i].position.z()
                    << " " << +vertices[i].color[0] << " " << +vertices[i].color[1]<< " " << +vertices[i].color[2]
                    << " " << +vertices[i].color[3] << std::endl;
        }
    }

    for(int i=0; i < nVertices; i++){
        if (i != nVertices - 1 and vertices[i].position.x() != MINF){
            if (i % 639 == 0 or i >= nVertices - width) continue;

            int corner_2 = i + width;
            int corner_3 = i + 1;
            //bool distTrue = isDistValid(vertices[corner_2], vertices[corner_3], edgeThreshold);
            bool distTrue = isDistValid(vertices[i], vertices[corner_2], vertices[corner_3], edgeThreshold);
            if (vertices[corner_2].position.x() != MINF and vertices[corner_3].position.x() != MINF and distTrue){
                outFile << "3 " << i << " " << corner_2 << " " << corner_3 << std::endl;
            } else continue;

            int corner_4 = corner_2 + 1;
            distTrue = isDistValid(vertices[corner_2], vertices[corner_3], vertices[corner_4], edgeThreshold);
            if (vertices[corner_4].position.x() != MINF and distTrue)
                outFile << "3 " << corner_2 << " " << corner_4 << " " << corner_3 << std::endl;
        }
    }

	// close file
	outFile.close();

	return true;
}

int main()
{
	// Make sure this path points to the data folder
    std::string filenameIn = "/Users/eralpkocas/Documents/TUM/3D Scanning & Motion Planning/Week 2/exercise_1_src/exercise_1_src/data/rgbd_dataset_freiburg1_xyz/";
	std::string filenameBaseOut = "mesh_";

	// load video
	std::cout << "Initialize virtual sensor..." << std::endl;
	VirtualSensor sensor;
	if (!sensor.Init(filenameIn))
	{
		std::cout << "Failed to initialize the sensor!\nCheck file path!" << std::endl;
		return -1;
	}

	// convert video to meshes
	while (sensor.ProcessNextFrame())
	{

        // get ptr to the current depth frame
		// depth is stored in row major (get dimensions via sensor.GetDepthImageWidth() / GetDepthImageHeight())
		float* depthMap = sensor.GetDepth();

		// get ptr to the current color frame
		// color is stored as RGBX in row major (4 byte values per pixel, get dimensions via sensor.GetColorImageWidth() / GetColorImageHeight())
		BYTE* colorMap = sensor.GetColorRGBX();

		// get depth intrinsics
		Matrix3f depthIntrinsics = sensor.GetDepthIntrinsics();
		float fX = depthIntrinsics(0, 0);
		float fY = depthIntrinsics(1, 1);
		float cX = depthIntrinsics(0, 2);
		float cY = depthIntrinsics(1, 2);

		// compute inverse depth extrinsics
		Matrix4f depthExtrinsicsInv = sensor.GetDepthExtrinsics().inverse();

		Matrix4f trajectory = sensor.GetTrajectory();
		Matrix4f trajectoryInv = sensor.GetTrajectory().inverse();

		// TODO 1: back-projection
		// write result to the vertices array below, keep pixel ordering!
		// if the depth value at idx is invalid (MINF) write the following values to the vertices array
		// vertices[idx].position = Vector4f(MINF, MINF, MINF, MINF);
		// vertices[idx].color = Vector4uc(0,0,0,0);
		// otherwise apply back-projection and transform the vertex to world space, use the corresponding color from the colormap
		Vertex* vertices = new Vertex[sensor.GetDepthImageWidth() * sensor.GetDepthImageHeight()];

		int imgWidth = sensor.GetDepthImageWidth();
		int imgHeight = sensor.GetDepthImageHeight();
        int numWH = imgWidth * imgHeight;
        for(int i=0; i < numWH; i++){
		    if(depthMap[i] == MINF){
		        vertices[i].position = Vector4f(MINF, MINF, MINF, MINF);
                vertices[i].color = Vector4uc(0,0,0,0);
            }
		    else{
		        int pixel_y = (i / (imgWidth - 1)) + 1;
                int pixel_x = i - (pixel_y - 1) * imgWidth;
                float currDepthValue = depthMap[i];
		        float camera_x = currDepthValue * ((float) pixel_x - cX) / fX;
                float camera_y = currDepthValue * ((float) pixel_y - cY) / fY;

                Vector4f camera_pos_vector = Vector4f(camera_x, camera_y, currDepthValue, (float) 1.0);
                Vector4f world_pos = (trajectoryInv * depthExtrinsicsInv * camera_pos_vector).transpose();

                unsigned char r = colorMap[4*i];
                unsigned char g = colorMap[4*i+1];
                unsigned char b = colorMap[4*i+2];
                unsigned char x = colorMap[4*i+3];

                vertices[i].position = world_pos;
                vertices[i].color = Vector4uc(r, g, b, x);

            }
		}

        // write mesh file
		std::stringstream ss;
		ss << filenameBaseOut << sensor.GetCurrentFrameCnt() << ".off";
		if (!WriteMesh(vertices, sensor.GetDepthImageWidth(), sensor.GetDepthImageHeight(), ss.str()))
		{
			std::cout << "Failed to write mesh!\nCheck file path!" << std::endl;
			return -1;
		}
		// free mem
		delete[] vertices;
	}

	return 0;
}
