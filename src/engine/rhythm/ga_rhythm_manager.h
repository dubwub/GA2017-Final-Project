// below are some headers for playing music on windows
#pragma comment(lib, "Winmm.lib")
#include <Windows.h>
#include <Mmsystem.h>
#include <mciapi.h>
#include <iostream>

// used for recording/reading beatmaps
#include <fstream>

#include "entity/ga_entity.h"
#include "physics/ga_intersection.tests.h"
#include "physics/ga_physics_component.h"
#include "physics/ga_physics_world.h"
#include "physics/ga_rigid_body.h"
#include "physics/ga_shape.h"
#include "framework/ga_sim.h"

// #defines to help organize some of the magic numbers flying around
#define NUM_CELEBRATION_BLOCKS 30 // number of blocks that explode when a hit occurs
#define BOX_CEILING 10.0f // this is where blocks spawn from
#define BOX_FLOOR -10.0f // this is where blocks are destroyed

// turn on PLAYBACK_MODE if you want to play the beatmap in beatmap.txt (note this functionality was a little rushed)
// maybe a touch buggy
#define PLAYBACK_MODE false

// ga_box_details helps organize all of the parts of an entity in the world
struct ga_box_details
{
	ga_entity* entity;
	ga_oobb* oobb;
	ga_physics_component* physics;

	float weight;
};

class ga_rhythm_manager
{
public:
	ga_rhythm_manager(int bpm, const std::string& songname, bool hard_mode, ga_physics_world* world, ga_sim* sim) : 
		_bpm(bpm), 
		_songname(songname),
		_song_bpm(60.0 * 1000.0 / _bpm),
		_excellent_hit_window(_song_bpm.count() / 20),
		_nice_hit_window(_song_bpm.count() / 10),
		_world(world),
		_sim(sim)
	{
		if (!PLAYBACK_MODE) // just in case we want to record a beatmap
		{
			_beatmap_file_write.open("beatmap.txt");
		}
		else
		{
			_beatmap_file_read.open("beatmap.txt");
			std::string line;
			while (std::getline(_beatmap_file_read, line))
			{
				_beatmap_beats.push_back(atoi(line.c_str()));
			}
			_beatmap_file_read.close();
		}

		// we can calculate how long a box takes (in seconds) to fall from the ceiling to the floor using
		// fall dist = 0.5 accel * fall-time^2
		// where accel = gravity / box_weight (because in this implementation, gravity is a force so acceleration scales with object mass)
		// we always want blocks to be falling from the top of the camera (BOX_CEILING), so we can adjust weight accordingly to make the visuals work
		// regardless of bpm
		float box_fall_time = _song_bpm.count() / 1000.0f;
		_box_weight = 0.5f * -world->get_gravity().axes[1] / (BOX_CEILING - BOX_FLOOR) * powf(box_fall_time, 2.0f); // derived from d = 1/2 at^2

		// we can then calculate the number of blocks that will ever be on the screen at the same point in time
		// using the box_fall_time and the bpm of the song, because we assume we spawn a bpm box per beat
		// we just need to know the number of beats 
		int max_active_blocks = ceil(box_fall_time / (_song_bpm.count() / 1000.0f)) + 1;
		// song_bpm is tracked in ms, sorry for magic numbers
		// but couldn't find a good way to convert to seconds without it truncating (duration_cast didn't seem to work and my google fu failed)
		// we also add 1 just to add a buffer because not too expensive

		// (if we're playing from a beatmap, the max_active_blocks is actually kind of high depending on the highest concentration of blocks)
		if (PLAYBACK_MODE)
		{
			// we'll determine the max concentration of blocks using a sliding window of max size = box_fall_time
			max_active_blocks = 0;
			for (int i = 0; i < _beatmap_beats.size(); i++)
			{
				int local_max = 1;
				for (int j = i; j < _beatmap_beats.size() && _beatmap_beats[j] - _beatmap_beats[i] <= _song_bpm.count(); j++)
				{
					local_max++;
				}
				if (local_max > max_active_blocks)
				{
					max_active_blocks = local_max;
				}
			}
		}
		// celebration blocks fall from the sky whenever you hit a note excellently! less for solid hits, none for misses
		for (int i = 0; i < NUM_CELEBRATION_BLOCKS; i++) {
			ga_entity* box = new ga_entity();
			box->set_translation({ 100.0f, 100.0f, 100.0f }); // garbage values, we really just need the blocks out of view

			ga_oobb* oobb = new ga_oobb();
			oobb->_half_vectors[0] = ga_vec3f::x_vector().scale_result(.25f);
			oobb->_half_vectors[1] = ga_vec3f::y_vector().scale_result(.25f);
			oobb->_half_vectors[2] = ga_vec3f::z_vector().scale_result(.25f);

			ga_physics_component* collider = new ga_physics_component(box, oobb, _box_weight * .5f);
			collider->get_rigid_body()->make_static(); // we don't want these boxes flying around while idle, so they're static until we need them

			world->add_rigid_body(collider->get_rigid_body());
			sim->add_entity(box);
			_celebration_boxes.push_back({ box, oobb, collider });
		}

		// all of our preconstructed blocks
		for (int i = 0; i < max_active_blocks; i++) {
			ga_entity* box = new ga_entity();
			box->set_translation({ 100.0f, 100.0f, 100.0f }); // garbage values, we really just need the blocks out of view

			ga_oobb* oobb = new ga_oobb();

			oobb->_half_vectors[0] = ga_vec3f::x_vector().scale_result(.75f);
			oobb->_half_vectors[1] = ga_vec3f::y_vector().scale_result(.75f);
			oobb->_half_vectors[2] = ga_vec3f::z_vector().scale_result(.75f);

			ga_physics_component* collider = new ga_physics_component(box, oobb, _box_weight);

			collider->get_rigid_body()->make_static(); // we don't want these boxes flying around while idle, so they're static until we need them

			world->add_rigid_body(collider->get_rigid_body());
			sim->add_entity(box);
			_bpm_boxes.push_back({ box, oobb, collider, _box_weight });
		}
	}

	void cleanup(); // clean up bpm blocks and celebratory blocks

	bool start_play(); // start playing the music
	void update_playhead(); // get the MCI timer
	void update_beat(); // update what beat we're on, also activates blocks as necessary
	void handle_press(); // to be called when the spacebar is pressed (not held), handles recording/calibration/regular mode
	void check_active_blocks(); // inactivates active blocks that have left the screen

	void set_calibrating_mode(bool do_calibrate); // not actually used tbh
	bool toggle_calibrating_mode(); // press c to toggle calibration
	bool toggle_recording_mode(); // press r to toggle recording

private:
	int _bpm; // bpm of the song
	std::string _songname; // song filename e.g. "8bitDungeonBoss.mp3"
	std::chrono::duration<float, std::milli> _song_bpm; // bpm in milliseconds
	
	ga_physics_world* _world; // world, sim pointers allow rhythm_manager to cleanup/deal with its own blocks
	ga_sim* _sim;

	// PRESS "C" FOR CALIBRATION TEST
	bool _calibrating = false;
	float _video_offset = 0.0f;
	float _audio_offset = 0.0f; // audio_offset = 100.0f is 100 ms of audio lag (these values can be figured out with calibration test)
	int _audio_calibration_instances = 0;

	// below are the currently running time since the beginning of the song visually and audially respectively
	std::chrono::duration<float, std::ratio<1, 1000>> _time_visual;
	std::chrono::duration<float, std::ratio<1, 1000>> _time_audio;
	
	float _excellent_hit_window, _nice_hit_window; // leniency on excellent hits and nice hits

	int _beat = -1; // what beat are we on?

	float _box_weight;

	// the falling blocks are going to be implemented on a rotating vector system where the user sees blocks being
	// created and destroyed, but we really just have a set number of blocks that we move around. this sort of optimization seems to be good
	// just because we want to avoid any sort of lag issues from actually destroying things, and translating blocks is really not a big deal
	int _block_head_index = 0; // start of the active block chain
	int _block_tail_index = 0; // one after last active block
	// (tail will always be >= the head, but remember that this is a rotating index so it might be near the beginning)

	std::vector<ga_box_details> _celebration_boxes;
	std::vector<ga_box_details> _bpm_boxes;

	// for recording beatmaps
	bool _recording = false;
	std::ofstream _beatmap_file_write;
	std::ifstream _beatmap_file_read;
	std::vector<int> _beatmap_beats;
};