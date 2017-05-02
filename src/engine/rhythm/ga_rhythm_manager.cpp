#include "rhythm/ga_rhythm_manager.h"

// returns true/false depending on successful start
bool ga_rhythm_manager::start_play() {
	// play the song
	TCHAR buf[128];
	DWORD err;
	std::string message = "open " + _songname + " type mpegvideo alias song";
	if ((err = mciSendString(message.c_str(), 0, 0, 0))) // requires mp3 file be in the same folder as exe
	{
		std::cout << err << std::endl;
		printf("Sequencer device did not open!\r\n");
		if (mciGetErrorString(err, &buf[0], sizeof(buf))) printf("%s\r\n", &buf[0]);
		return false;
	}
	if ((err = mciSendString("play song", 0, 0, 0)))
	{
		std::cout << err << std::endl;
		printf("Song did not play!\r\n");
		if (mciGetErrorString(err, &buf[0], sizeof(buf))) printf("%s\r\n", &buf[0]);
		return false;
	}
	return true;
}

void ga_rhythm_manager::update_playhead()
{
	// some small helper variables used with mcisendstring in order to get running playhead
	DWORD error;
	LPSTR mci_output = new char[128]; // maybe in the future make this a class var so this isn't being free/unfreed so often

	// just to make sure everything is synced up with the actual song that's playing, we'll use MCI 
	error = mciSendString("status song position", mci_output, 128, 0);
	int visual_milliseconds_since_start = atoi(mci_output);
	int audio_milliseconds_since_start = visual_milliseconds_since_start;
	if (!_calibrating)
	{
		audio_milliseconds_since_start -= _audio_offset;
		visual_milliseconds_since_start -= _video_offset;
	}

	_time_visual = std::chrono::duration<float, std::ratio<1, 1000>>(visual_milliseconds_since_start); // actually update playheads
	_time_audio = std::chrono::duration<float, std::ratio<1, 1000>>(audio_milliseconds_since_start);

	delete[] mci_output;
}

/*
	if we've passed the most previous beat, we update the beat counter and create a new block
*/
void ga_rhythm_manager::update_beat()
{
	if (!PLAYBACK_MODE)
	{
		if ((int)(_time_visual.count() / _song_bpm.count()) > _beat)
		{
			_beat = (int)(_time_visual.count() / _song_bpm.count());
			float time_till_beat = (_beat + 1) * _song_bpm.count() - _time_visual.count();

			// there may be a better way of converting from ms to s, but duration_cast seemed to truncate
			// or round and created inaccurate results

			_bpm_boxes[_block_tail_index].entity->set_translation(
			{
				rand() % 20 - 10.0f,
				0.5f * (-_world->get_gravity().axes[1] / _bpm_boxes[_block_tail_index].weight) * powf(time_till_beat / 1000.0f, 2.0f), // note: gravity is negative
				rand() % 20 - 10.0f
			});
			_bpm_boxes[_block_tail_index].physics->get_rigid_body()->make_not_static();
			_block_tail_index++;
			_block_tail_index %= _bpm_boxes.size();
		}
	}
	else
	{
		while (_beat == -1 || (_beat < _beatmap_beats.size() - 1 && _time_visual.count() > _beatmap_beats[_beat]))
		{
			_beat++;
			float time_till_beat = _beatmap_beats[_beat] - _time_visual.count();
			_bpm_boxes[_block_tail_index].entity->set_translation(
			{
				rand() % 20 - 10.0f,
				0.5f * (-_world->get_gravity().axes[1] / _bpm_boxes[_block_tail_index].weight) * powf(time_till_beat / 1000.0f, 2.0f), // note: gravity is negative
				rand() % 20 - 10.0f
			});
			_bpm_boxes[_block_tail_index].physics->get_rigid_body()->make_not_static();
			_block_tail_index++;
			_block_tail_index %= _bpm_boxes.size();
		}
	}
}

// this gets called with each space bar press, we calculate how off they are using the audio_offset
void ga_rhythm_manager::handle_press()
{
	float time_till_last_beat, time_till_beat;
	if (!PLAYBACK_MODE)
	{
		time_till_last_beat = _time_audio.count() - _beat * _song_bpm.count();
		time_till_beat = (_beat + 1) * _song_bpm.count() - _time_audio.count();
	}
	else
	{
		time_till_last_beat = _time_audio.count() - _beatmap_beats[_beat];
		time_till_beat = _beatmap_beats[_beat + 1] - _time_audio.count();
	}

	if (_calibrating) // if we're calibrating, don't use audio offset, just adjust it
	{
		// TODO: actually set visual_offset
		std::cout << "visual: " << _bpm_boxes[_block_head_index].entity->get_translation().axes[1] << std::endl;
		_audio_offset = (_audio_offset * _audio_calibration_instances + time_till_last_beat) / (_audio_calibration_instances + 1);
		_audio_calibration_instances++;
		std::cout << "audio_offset: " << _audio_offset << std::endl;
	}
	else // not calibrating, time to go nuts
	{
		if (_recording)
		{
			_beatmap_file_write << _time_audio.count() << std::endl;
		}
		if (time_till_beat < _excellent_hit_window || time_till_last_beat < _excellent_hit_window) // excellent hit!
		{
			for (int i = 0; i < _celebration_boxes.size(); i++)
			{
				_celebration_boxes[i].entity->set_translation(
				{
					_bpm_boxes[_block_head_index].entity->get_translation().axes[0],
					0.0f,
					_bpm_boxes[_block_head_index].entity->get_translation().axes[2]
				});
				_celebration_boxes[i].physics->get_rigid_body()->add_linear_velocity({ rand() % 20 - 1.0f, rand() % 80 * 1.0f, rand() % 20 - 10.0f });
				_celebration_boxes[i].physics->get_rigid_body()->make_not_static();
			}
		}
		else if (time_till_beat < _nice_hit_window || time_till_last_beat < _nice_hit_window) // nice hit
		{
			for (int i = 0; i < _celebration_boxes.size() / 2; i++)
			{
				_celebration_boxes[i].entity->set_translation(
				{
					_bpm_boxes[_block_head_index].entity->get_translation().axes[0],
					0.0f,
					_bpm_boxes[_block_head_index].entity->get_translation().axes[2]
				});
				_celebration_boxes[i].physics->get_rigid_body()->add_linear_velocity({ rand() % 5 - 2.5f, rand() % 40 * 1.0f, rand() % 5 - 2.5f });
				_celebration_boxes[i].physics->get_rigid_body()->make_not_static();
			}
		}
		else
		{
			// missed hit, maybe something happens?
			std::cout << ":(" << std::endl;
		}
	}
}

void ga_rhythm_manager::check_active_blocks()
{
	// .data[4][1] is the y-coordinate of the entity (is there a better way of doing this??)
	// here we update the blockhead if the current blockhead's y-position is below our block floor
	// note that because of the order in which blocks are dropped from the ceiling, the blockhead is guaranteed
	// to be the lowest y-position of all active blocks
	while (_bpm_boxes[_block_head_index].entity->get_translation().axes[1] < BOX_FLOOR)
	{
		_bpm_boxes[_block_head_index].entity->set_translation({ 100.0f, 100.0f, 100.0f });
		_bpm_boxes[_block_head_index].physics->get_rigid_body()->make_static();
		_bpm_boxes[_block_head_index].physics->get_rigid_body()->reset_speeds();

		_block_head_index++;
		_block_head_index %= _bpm_boxes.size();
	}

	// constant time operation, not too expensive so it's ok
	for (int i = 0; i < _celebration_boxes.size(); i++)
	{
		if (_celebration_boxes[i].entity->get_translation().axes[1] < BOX_FLOOR)
		{
			_celebration_boxes[i].entity->set_translation({ 100.0f, 100.0f, 100.0f });
			_celebration_boxes[i].physics->get_rigid_body()->make_static();
			_celebration_boxes[i].physics->get_rigid_body()->reset_speeds();
		}
	}
}

// while in calibrating mode, the game continues running the video/audio as normal
// but instead of awarding celebration explosions, it simply calculates audio/video offset
void ga_rhythm_manager::set_calibrating_mode(bool do_calibrate)
{
	_calibrating = do_calibrate;
}

bool ga_rhythm_manager::toggle_calibrating_mode()
{
	_calibrating = !_calibrating;
	return _calibrating;
}

bool ga_rhythm_manager::toggle_recording_mode()
{
	_recording = !_recording;
	return _recording;
}

void ga_rhythm_manager::cleanup()
{
	for (int i = 0; i < _bpm_boxes.size(); i++)
	{
		_world->remove_rigid_body(_bpm_boxes[i].physics->get_rigid_body());
		delete _bpm_boxes[i].entity;
		delete _bpm_boxes[i].oobb;
		delete _bpm_boxes[i].physics;
	}

	for (int i = 0; i < _celebration_boxes.size(); i++)
	{
		_world->remove_rigid_body(_celebration_boxes[i].physics->get_rigid_body());
		delete _celebration_boxes[i].entity;
		delete _celebration_boxes[i].oobb;
		delete _celebration_boxes[i].physics;
	}

	if (!PLAYBACK_MODE)
	{
		_beatmap_file_write.close();
	}
	// the reading file is closed way back in the constructor, no need to clean up again
}