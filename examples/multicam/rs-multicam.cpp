// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2015-2017 Intel Corporation. All Rights Reserved.

#include <librealsense2/rs.hpp>     // Include RealSense Cross Platform API
#include "example.hpp"              // Include short list of convenience functions for rendering

#include <string>
#include <map>
#include <algorithm>
#include <mutex>                    // std::mutex, std::lock_guard
#include <cmath>                    // std::ceil
#include <fstream>

const std::string no_camera_message = "No camera connected, please connect 1 or more";
const std::string platform_camera_name = "Platform Camera";

class device_container
{
    // Helper struct per pipeline
    struct view_port
    {
        std::map<int, rs2::frame> frames_per_stream;
        rs2::colorizer colorize_frame;
        texture tex;
        rs2::pipeline pipe;
        rs2::pipeline_profile profile;
		rs2::stream_profile stream_profile;
    };

public:

	state pointMarkers;
	device_container(state pointMarker) {
		pointMarkers = pointMarker;
	}

    void enable_device(rs2::device dev)
    {
        std::string serial_number(dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER));
        std::lock_guard<std::mutex> lock(_mutex);

        if (_devices.find(serial_number) != _devices.end())
        {
            return; //already in
        }
		
        // Ignoring platform cameras (webcams, etc..)
        if (platform_camera_name == dev.get_info(RS2_CAMERA_INFO_NAME))
        {
            return;
        }
        // Create a pipeline from the given device
        rs2::pipeline p;
        rs2::config c;
        c.enable_device(serial_number);
        // Start the pipeline with the configuration
        rs2::pipeline_profile profile = p.start(c);
	
        // Hold it internally
        _devices.emplace(serial_number, view_port{ {},{},{}, p, profile });
		

    }

    void remove_devices(const rs2::event_information& info)
    {
        std::lock_guard<std::mutex> lock(_mutex);
        // Go over the list of devices and check if it was disconnected
        auto itr = _devices.begin();
        while(itr != _devices.end())
        {
            if (info.was_removed(itr->second.profile.get_device()))
            {
                itr = _devices.erase(itr);
            }
            else
            {
                ++itr;
            }
        }
    }

    size_t device_count()
    {
        std::lock_guard<std::mutex> lock(_mutex);
        return _devices.size();
    }

    int stream_count()
    {
        std::lock_guard<std::mutex> lock(_mutex);
        int count = 0;
        for (auto&& sn_to_dev : _devices)
        {
            for (auto&& stream : sn_to_dev.second.frames_per_stream)
            {
                if (stream.second)
                {
                    count++;
                }
            }
        }
        return count;
    }

	void mergeFrames(rs2::depth_frame& frame1, rs2::depth_frame& frame2)
	{
		//const int frameWidth = frame1.get_width();
		//const int frameHeight = frame1.get_height();
		//std::cout << frameHeight << " - " << frameWidth;
	}

	void poll_frames()
	{
		std::lock_guard<std::mutex> lock(_mutex);
		// Go over all device
		int number_of_devices = _devices.size();
		int current_device = 0;

		rs2::depth_frame frames[3];
		rs2::stream_profile profiles[3];

        for (auto&& view : _devices)
        {
            // Ask each pipeline if there are new frames available

			rs2::device selected_device = view.second.profile.get_device();
			auto depth_sensor = selected_device.first<rs2::depth_sensor>();
			
            rs2::frameset frameset;
            if (view.second.pipe.poll_for_frames(&frameset))
            {
				frames[current_device] = frameset.get_depth_frame();
                for (int i = 0; i < frameset.size(); i++)
                {
                    rs2::frame new_frame = frameset[i];
                    int stream_id = new_frame.get_profile().unique_id();
                    view.second.frames_per_stream[stream_id] = view.second.colorize_frame(new_frame); //update view port with the new stream
                }
				
				profiles[current_device] = frames[current_device].get_profile();
				//std::vector<std::vector<float>> vec1 = getPixelCloud(frames[current_device]);
				if (current_device ==0) {
					writeVerticesToCsv(frames[current_device]);
				    exit(9);
				}

            }
			current_device++;
        }
		mergeFrames(frames[0], frames[1]);

    }


	void render_textures(int cols, int rows, float view_width, float view_height)
	{
		std::lock_guard<std::mutex> lock(_mutex);
		int stream_no = 0;
		
		for (auto&& view : _devices)
		{
			// For each device get its frames
			for (auto&& id_to_frame : view.second.frames_per_stream)
			{
				// If the frame is available
				if (id_to_frame.second)
				{
					view.second.tex.upload(id_to_frame.second);
				}
				rect frame_location{ view_width * (stream_no % cols), view_height * (stream_no / cols), view_width, view_height };
				if (rs2::depth_frame vid_frame = id_to_frame.second.as<rs2::depth_frame>())
				{
					rect adjuested = frame_location.adjust_ratio({ static_cast<float>(vid_frame.get_width())
						, static_cast<float>(vid_frame.get_height()) });

					view.second.tex.show(adjuested);
					switch (stream_no) {
					case 1:
						float marker_point_1 = pointMarkers.point1A.y;
						std::cout <<"Point 1 =" <<marker_point_1 << std::endl;
						break;
					}

					stream_no++;
				}
			}
		}
	}

	void writeVerticesToCsv(rs2::depth_frame frame)
	{
		std::stringstream csv_file;
		csv_file << "cam2.csv";
		metadata_to_csv(frame, csv_file.str());
	}

	void metadata_to_csv(const rs2::depth_frame frame, const std::string& filename)
	{
		std::ofstream csv;

		csv.open(filename);
		rs2::pointcloud cloud;
		rs2::points points = cloud.calculate(frame);

		auto vertices = points.get_vertices();
		
		
		for (int i = 0; i <= 640; i++)
		{
			for (int j = 0; j <= 480; j++) {
				/*if (vertices[i].z)
				{
					csv << vertices[i].x << "," << vertices[i].y << "," << vertices[i].z << "\n";
				}*/

				csv << i << "," << j << "," << frame.get_distance(i,j) << "\n";
			}
			
		}

		csv.close();
	}


    
private:
    std::mutex _mutex;
    std::map<std::string, view_port> _devices;
};
// Helper function to register to UI events
void register_glfw_callbacks(window& app, state& app_state);


int main(int argc, char * argv[]) try
{
	int call_count = 0;
    // Create a simple OpenGL window for rendering:
    window app(1280, 960, "REALSENSE OF HUMOUR");
	state point_markers;

	device_container connected_devices(point_markers);

	point_markers.point1A = { 0.15f, 0.5f };
	point_markers.point1B = { 0.25f, 0.5f };

	point_markers.point2A = { 0.45f, 0.5f };
	point_markers.point2B = { 0.55f, 0.5f };

	point_markers.point3A = { 0.75f, 0.5f };
	point_markers.point3B = { 0.85f, 0.5f };

	register_glfw_callbacks(app, point_markers);


    rs2::context ctx;    // Create librealsense context for managing devices

                         // Register callback for tracking which devices are currently connected
    ctx.set_devices_changed_callback([&](rs2::event_information& info)
    {
        connected_devices.remove_devices(info);
        for (auto&& dev : info.get_new_devices())
        {
            connected_devices.enable_device(dev);
        }
    });

    // Initial population of the device list
    for (auto&& dev : ctx.query_devices()) // Query the list of connected RealSense devices
    {
        connected_devices.enable_device(dev);
    }

    while (app) // Application still alive?
    {
        connected_devices.poll_frames();
        auto total_number_of_streams = connected_devices.stream_count();
        if (total_number_of_streams == 0)
        {
            draw_text(int(std::max(0.f, (app.width() / 2) - no_camera_message.length() * 3)),
                      int(app.height() / 2), no_camera_message.c_str());
            continue;
        }
        if (connected_devices.device_count() == 1)
        {
            draw_text(0, 10, "Please connect another camera");
        }

        int cols = int(std::ceil(std::sqrt(total_number_of_streams)));
        int rows = int(std::ceil(total_number_of_streams / static_cast<float>(cols)));

        float view_width = (app.width() / cols);
        float view_height = (app.height() / rows);

		//std::cout << "view_height = " << view_height << " view_width = " << view_width <<"rows"<<rows<<"  coulumns = "<<cols<< std::endl;
        connected_devices.render_textures(cols, rows, view_width, view_height);
		{
		
			// Use 1-Color model to invert background colors
			//glBlendFunc(GL_ONE_MINUS_DST_COLOR, GL_ONE_MINUS_SRC_COLOR);

			// Render the ruler
			point_markers.point1A.render_shape(app, 3);
			point_markers.point1B.render_shape(app, 4);

			point_markers.point2A.render_shape(app, 3);
			point_markers.point2B.render_shape(app, 4);

			point_markers.point3A.render_shape(app, 3);
			point_markers.point3B.render_shape(app, 4);

			glColor3f(1.f, 1.f, 1.f);
		}
		call_count++;
    }

    return EXIT_SUCCESS;
}
catch (const rs2::error & e)
{
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
}
catch (const std::exception & e)
{
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}
// Implement drag&drop behaviour for the buttons:
void register_glfw_callbacks(window& app, state& app_state)
{
	app.on_left_mouse = [&](bool pressed)
	{
		app_state.mouse_down = pressed;
	};

	app.on_mouse_move = [&](double x, double y)
	{
		toggle cursor{ float(x) / app.width(), float(y) / app.height() };
		std::vector<toggle*> toggles{
			&app_state.point1A,
			&app_state.point1B,
			&app_state.point2A,
			&app_state.point2B,
			&app_state.point3A,
			&app_state.point3B
		};

		if (app_state.mouse_down)
		{
			toggle* best = toggles.front();
			for (auto&& t : toggles)
			{
				if (t->dist_2d(cursor) < best->dist_2d(cursor))
				{
					best = t;
				}
			}
			best->selected = true;
		}
		else
		{
			for (auto&& t : toggles) t->selected = false;
		}

		for (auto&& t : toggles)
		{
			if (t->selected) *t = cursor;
		}
	};
}
