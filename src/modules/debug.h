#pragma once

#include <sstream>
#include <string>
#include <fstream>
#include <opencv2/freetype.hpp>

#include "../constants.h"

// interface

namespace debug
{

	constexpr auto DOUT_PRECISION = 4;
	constexpr auto MARGIN_L       = 10;
	constexpr auto FONT_SIZE      = 15;
	constexpr auto FONT_THICK     = -1;
	constexpr auto LINE_H         = FONT_SIZE;
	constexpr auto CHAR_W         = 0.f;
	const     auto FONT_COLOR     = cv::Scalar::all(255);
	constexpr auto WNDW_DEBUG_H   = 400;
	constexpr auto WNDW_DEBUG_W   = 700;

	static std::stringstream cout;
	static std::stringstream dout;
	static std::string wndw_name = "debug";	

	void
	show();

	void
	show(void (*callback)());
}

// implementation

inline void
debug::show()
{
	// dump stream
	static bool init = true;
	static std::ofstream dump_stream;
	if(init) {dump_stream.open(PATH_ASSETS + "dump.txt"); init = false;}
	else dump_stream.open(PATH_ASSETS + "dump.txt", std::ios_base::app);

	// image
	static cv::Mat img_debug = cv::Mat(WNDW_DEBUG_H, WNDW_DEBUG_W, CV_8UC3);
	img_debug.setTo(0);

	// font
	// https://docs.opencv.org/4.1.1/d9/dfa/classcv_1_1freetype_1_1FreeType2.html
	static cv::Ptr<cv::freetype::FreeType2> font = cv::freetype::createFreeType2();
	font->loadFontData(PATH_FONT_CONSOLAS, 0);
	
	// create a stream for output
	std::stringstream out;

	// apply settings to debug stream
	//debug::dout.setf(std::ios::fixed, std::ios::floatfield);
	debug::dout.flags(std::ios::showpos | std::ios::fixed);
	debug::dout.precision(DOUT_PRECISION);

	// fill with default data
	out << "debugger:\n\n";

	// append global data
	out << debug::dout.str();
	out << '\n';
	out << debug::cout.str();	

	// write to dump
	dump_stream << out.str() << "\n";
	dump_stream.close();
	
	// clear global stream for next iteration
	debug::cout.str(""); debug::cout.clear();
	debug::dout.str(""); debug::dout.clear();

	// put lines of text onto image
	for (auto [i, line] = std::tuple{ 1, std::string() }; std::getline(out, line); i++)
		font->putText(img_debug, line, cv::Point(MARGIN_L, LINE_H * i), FONT_SIZE, FONT_COLOR, FONT_THICK, 8, true);

	// draw image
	cv::imshow(wndw_name, img_debug);
}

inline void
debug::show(void (*callback)())
{
	// populate data
	callback();

	// show window
	debug::show();
}

