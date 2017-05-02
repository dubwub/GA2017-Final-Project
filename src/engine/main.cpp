/*
** RPI Game Architecture Engine
**
** Portions adapted from:
** Viper Engine - Copyright (C) 2016 Velan Studios - All Rights Reserved
**
** This file is distributed under the MIT License. See LICENSE.txt.
*/

// play around with these pound defines for other functionality
#define HARD_MODE false // HARD_MODE was a cheeky little thing but had to disable a lot of functionality to get it working :/

#include "framework/ga_camera.h"
#include "framework/ga_compiler_defines.h"
#include "framework/ga_input.h"
#include "framework/ga_sim.h"
#include "framework/ga_output.h"
#include "jobs/ga_job.h"

#include "entity/ga_entity.h"

#include "graphics/ga_cube_component.h"
#include "graphics/ga_program.h"

#include "physics/ga_intersection.tests.h"
#include "physics/ga_physics_component.h"
#include "physics/ga_physics_world.h"
#include "physics/ga_rigid_body.h"
#include "physics/ga_shape.h"

#include "rhythm/ga_rhythm_manager.h"

#define STB_IMAGE_IMPLEMENTATION
#include <stb_image.h>

#define STB_TRUETYPE_IMPLEMENTATION
#include <stb_truetype.h>

#include <iostream>
#include <cstdlib> // for atoi

#if defined(GA_MINGW)
#include <unistd.h>
#endif

static void set_root_path(const char* exepath);
static void run_unit_tests();

int main(int argc, const char** argv)
{
	set_root_path(argv[0]);

	ga_job::startup(0xffff, 256, 256);

	// Create objects for three phases of the frame: input, sim and output.
	ga_input* input = new ga_input();
	ga_sim* sim = new ga_sim();
	ga_physics_world* world = new ga_physics_world();
	ga_output* output = new ga_output(input->get_window());

	// Create camera.
	ga_camera* camera = new ga_camera({ 0.0f, 7.0f, 20.0f });
	ga_quatf rotation;
	rotation.make_axis_angle(ga_vec3f::y_vector(), ga_degrees_to_radians(180.0f));
	camera->rotate(rotation);
	rotation.make_axis_angle(ga_vec3f::x_vector(), ga_degrees_to_radians(15.0f));
	camera->rotate(rotation);

	// init the manager
	int bpm = 134;
	if (HARD_MODE)
	{
		bpm *= 2;
	}
	ga_rhythm_manager manager(bpm, std::string("8bitDungeonBoss.mp3"), false, world, sim); // hardcoded
	manager.start_play();

	// floor here isn't for collisions, just shows when players should hit the spacebar
	ga_entity floor;
	ga_plane floor_plane;
	floor_plane._point = { 0.0f, 0.0f, 0.0f };
	floor_plane._normal = { 0.0f, 1.0f, 0.0f };
	ga_physics_component floor_collider(&floor, &floor_plane, 0.0f);
	floor_collider.get_rigid_body()->make_static();
	world->add_rigid_body(floor_collider.get_rigid_body());
	sim->add_entity(&floor);

	// space is how the user attempts to hit a rhythm window
	// we do not want to penalize a player for holding space, because that would feel bad
	// so by using a boolean we can figure out whether the player has pressed or held space
	// in an ideal world there is a space bar listener as opposed to checking whether they're holding every frame
	// we make the same variable for c as well just because we use c for calibration
	// we do the same for r for recording
	bool holding_space = false;
	bool holding_c = false;
	bool holding_r = false;
	
	// Main loop
	while (true)
	{
		// We pass frame state through the 3 phases using a params object.
		ga_frame_params params;
		
		// Gather user input and current time.
		if (!input->update(&params))
		{
			break;
		}

		// Update the camera.
		camera->update(&params);
		// Run gameplay.
		sim->update(&params);
		// Step the physics world.
		world->step(&params);
		// Perform the late update.
		sim->late_update(&params);
		// Draw to screen.
		output->update(&params);

		manager.update_playhead();
		manager.update_beat();

		if ((params._button_mask & k_button_c) == k_button_c)
		{
			if (!holding_c)
			{
				holding_c = true;
				manager.toggle_calibrating_mode(); // pressing c toggles between calibrating
			}
		}
		else
		{
			holding_c = false;
		}

		if ((params._button_mask & k_button_r) == k_button_r)
		{
			if (!holding_r)
			{
				holding_r = true;
				manager.toggle_recording_mode();
			}
		}

		if ((params._button_mask & k_button_space) == k_button_space) // check for whether they press space
		{
			if (!holding_space)
			{
				holding_space = true;
				manager.handle_press();
			}
		}
		else
		{
			holding_space = false;
		}

		manager.check_active_blocks();
	}

	manager.cleanup();

	world->remove_rigid_body(floor_collider.get_rigid_body());
	
	delete output;
	delete world;
	delete sim;
	delete input;
	delete camera;

	ga_job::shutdown();

	return 0;
}

char g_root_path[256];
static void set_root_path(const char* exepath)
{
#if defined(GA_MSVC)
	strcpy_s(g_root_path, sizeof(g_root_path), exepath);

	// Strip the executable file name off the end of the path:
	char* slash = strrchr(g_root_path, '\\');
	if (!slash)
	{
		slash = strrchr(g_root_path, '/');
	}
	if (slash)
	{
		slash[1] = '\0';
	}
#elif defined(GA_MINGW)
	char* cwd;
	char buf[PATH_MAX + 1];
	cwd = getcwd(buf, PATH_MAX + 1);
	strcpy_s(g_root_path, sizeof(g_root_path), cwd);

	g_root_path[strlen(cwd)] = '/';
	g_root_path[strlen(cwd) + 1] = '\0';
#endif
}

void run_unit_tests()
{
	ga_intersection_utility_unit_tests();
	ga_intersection_unit_tests();
}
