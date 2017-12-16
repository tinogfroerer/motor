#pragma once

typedef struct _chuckState {
	float x;
	float y;
	bool valid;
	_chuckState() : valid(false) {}
	_chuckState(const float _x, const float _y) : x(_x), y(_y), valid(true) {}
} chuckState;
