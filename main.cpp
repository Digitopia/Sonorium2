// This file is part of the Orbbec Astra SDK [https://orbbec3d.com]
// Copyright (c) 2015-2017 Orbbec 3D
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
// http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Be excellent to each other.
#include <SFML/Graphics.hpp>

#include "boost/asio.hpp"

#include <astra_core/astra_core.hpp>
#include <astra/astra.hpp>
#include "LitDepthVisualizer.hpp"
#include <chrono>
#include <iostream>
#include <sstream>
#include <iomanip>
#include <key_handler.h>


#include <boost/chrono.hpp>
#include <boost/thread/thread.hpp> 
#include <boost/array.hpp>
#include <boost/asio.hpp>
#include <boost/bind.hpp>

#include <map>
#include <set>

int mouseX_{ 0 };
int mouseY_{ 0 };
float mouseWorldX_{ 0 };
float mouseWorldY_{ 0 };
float mouseWorldZ_{ 0 };
bool isPaused_{ false };
bool isMouseOverlayEnabled_{ true };

#include <boost/thread/thread.hpp>

using namespace std;
using namespace boost::asio;

io_service ios123;
io_service ios_receive;
ip::udp::socket socket123(ios123);
ip::udp::endpoint remote_endpoint;
boost::system::error_code err;

// receive
ip::udp::socket socket_receive{ios_receive};
ip::udp::endpoint receive_endpoint;
boost::system::error_code receive_err;
boost::array<char, 1024> recv_buf;

// Variáveis Configuração

string SEND_IP = "127.0.0.1";
int SEND_PORT = 9000;
int RECEIVE_PORT = 9005;

std::map<char, int> GRID_SIZE = {
	{ 'x', 4 }, // distancia lateral em relação ao centro e.g. [-2000, 2000]
	{ 'y', 1 }, // altura em relação ao centro e.g. [-2000, 2000]
	{ 'z', 4 }, // profundidade e.g. [0, 7000]
};

// milimeters
std::map<string, float> CAPTURE_AREA = {
	{ "xMin", -1200.0 },
	{ "xMax",  1200.0 },
	{ "yMin", -1200.0 },
	{ "yMax",  1200.0 },
	{ "zMin",  2000.0 },
	{ "zMax",  4000.0 },
};

float GRID_GAP = 300; // in mm

void handler_osc_receive(
	const boost::system::error_code& error, // Result of operation.
	std::size_t bytes_transferred           // Number of bytes received.
);

void handler_osc_receive(const boost::system::error_code & error, std::size_t bytes_transferred)
{
	std::cout << "receive something!";
}

vector<string> split(const string& str, const string& delim)
{
	vector<string> tokens;
	size_t prev = 0, pos = 0;
	do
	{
		pos = str.find(delim, prev);
		if (pos == string::npos) pos = str.length();
		string token = str.substr(prev, pos - prev);
		if (!token.empty()) tokens.push_back(token);
		prev = pos + delim.length();
	} while (pos < str.length() && prev < str.length());
	return tokens;
}

struct Client {
    
	boost::asio::io_service io_service;
    ip::udp::socket socket{io_service};
    boost::array<char, 2048> recv_buffer;
    ip::udp::endpoint remote_endpoint;

    void handle_receive(const boost::system::error_code& error, size_t bytes_transferred) {
        if (error) {
            std::cout << "Receive failed: " << error.message() << "\n";
            return;
        }
		std::string received = string(recv_buffer.begin(), recv_buffer.begin() + bytes_transferred);
        std::cout << "Received: '" << received << "' (" << error.message() << ")\n";
		
		// std::string delimiter = " ";
		// std::string cmd = received.substr(0, received.find(delimiter));

		vector<string> tokens = split(received, " ");

		// NOTE: make sure that the buffer doesn't get full so that can keep on receiving messages
		recv_buffer.assign(0);

		std::string cmd = tokens[0];

		if (cmd == "grid_size") {
			std::cout << "grid_size" << std::endl;
			GRID_SIZE['x'] = std::stoi(tokens[1]);
			GRID_SIZE['y'] = std::stoi(tokens[2]);
			GRID_SIZE['z'] = std::stoi(tokens[3]);
		}
		else if (cmd == "grid_gap") {
			std::cout << "grid_gap" << std::endl;
			GRID_GAP = std::stoi(tokens[1]);
		}
		else if (cmd == "capture_area") {
			std::cout << "capture_area" << std::endl;
			CAPTURE_AREA["xMin"] = std::stoi(tokens[1]);
			CAPTURE_AREA["xMax"] = std::stoi(tokens[2]);
			CAPTURE_AREA["yMin"] = std::stoi(tokens[3]);
			CAPTURE_AREA["yMax"] = std::stoi(tokens[4]);
			CAPTURE_AREA["zMin"] = std::stoi(tokens[5]);
			CAPTURE_AREA["zMax"] = std::stoi(tokens[6]);
		}
		else {
			std::cout << "Comand nao suportado... Utilizar grid_size, grid_gap ou capture_area" << std::endl;
		}

        wait();
        
    }

    void wait() {
        socket.async_receive_from(boost::asio::buffer(recv_buffer),
            remote_endpoint,
            boost::bind(&Client::handle_receive, this, boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));
    }

    void Receiver()
    {
        socket.open(ip::udp::v4());
        socket.bind(ip::udp::endpoint(ip::address::from_string(SEND_IP), RECEIVE_PORT));

        wait();

        std::cout << "Receiving\n";
        io_service.run();
        std::cout << "Receiver exit\n";
    }
};

float ofMap(float value, float inputMin, float inputMax, float outputMin, float outputMax, bool clamp) {

	if (fabs(inputMin - inputMax) < FLT_EPSILON) {
		std::cout << "ofMap(): avoiding possible divide by zero, check inputMin and inputMax: " << inputMin << " " << inputMax;
		return outputMin;
	}
	else {
		float outVal = ((value - inputMin) / (inputMax - inputMin) * (outputMax - outputMin) + outputMin);

		if (clamp) {
			if (outputMax < outputMin) {
				if (outVal < outputMax)outVal = outputMax;
				else if (outVal > outputMin)outVal = outputMin;
			}
			else {
				if (outVal > outputMax)outVal = outputMax;
				else if (outVal < outputMin)outVal = outputMin;
			}
		}
		return outVal;
	}

}


unsigned int getPositionInGrid(float val, float gridStep, float gridGap) {
	gridStep += 0.01; // NOTE: so that if there are 4 steps e.g. doesn't return 5 possible values but 4 instead as expected
	const unsigned int ret = floor(val / (gridStep / 2));
	return ret + 1;
}

int getPositionInGridGap(float val, float gridStep, unsigned int gridSize, float gridGap) {
	gridStep += 0.01; // NOTE: so that if there are 4 steps e.g. doesn't return 5 possible values but 4 instead as expected
	// const unsigned int ret = floor(val / (gridStep / 2));
	for (int i = 0; i < gridSize; i++) {
		const float limStart = i * gridStep + gridGap * i;
		const float limEnd = limStart + gridStep;
		//cout << "limStart " << limStart << " limEnd" << limEnd << " ";
		if (val >= limStart && val < limEnd) {
			return i + 1; // NOTE: so that indices are not 0-based but instead 1-based
		}
	}
	return -1; // NOTE: return in grid gap area
}

enum ColorMode
{
	MODE_COLOR,
	MODE_IR_16,
	MODE_IR_RGB,
};

class MultiFrameListener : public astra::FrameListener
{
public:
	using BufferPtr = std::unique_ptr<uint8_t[]>;

	struct StreamView
	{
		sf::Sprite sprite_;
		sf::Texture texture_;
		BufferPtr buffer_;
		int width_{ 0 };
		int height_{ 0 };
	};

	MultiFrameListener()
	{
		prev_ = ClockType::now();
	}

	void init_texture(int width, int height, StreamView& view)
	{
		if (view.buffer_ == nullptr || width != view.width_ || height != view.height_)
		{
			view.width_ = width;
			view.height_ = height;

			// texture is RGBA
			const int byteLength = width * height * 4;

			view.buffer_ = BufferPtr(new uint8_t[byteLength]);

			clear_view(view);

			view.texture_.create(width, height);
			view.sprite_.setTexture(view.texture_, true);
			view.sprite_.setPosition(0, 0);
		}
	}

	void clear_view(StreamView& view)
	{
		const int byteLength = view.width_ * view.height_ * 4;
		std::fill(&view.buffer_[0], &view.buffer_[0] + byteLength, 0);
	}

	void check_fps()
	{
		const float frameWeight = .2f;

		const ClockType::time_point now = ClockType::now();
		const float elapsedMillis = std::chrono::duration_cast<DurationType>(now - prev_).count();

		elapsedMillis_ = elapsedMillis * frameWeight + elapsedMillis_ * (1.f - frameWeight);
		prev_ = now;

		const float fps = 1000.f / elapsedMillis;

		const auto precision = std::cout.precision();

		std::cout << std::fixed
			<< std::setprecision(1)
			<< fps << " fps ("
			<< std::setprecision(1)
			<< elapsedMillis_ << " ms)"
			<< std::setprecision(precision)
			<< std::endl;
	}

	
	void update_depth(astra::Frame& frame)
	{
		const astra::PointFrame pointFrame = frame.get<astra::PointFrame>();

		if (pointFrame.is_valid())
		{
			/*clear_view(depthView_);
			depthView_.texture_.update(&depthView_.buffer_[0]);
			return;*/

			int width = pointFrame.width();
			int height = pointFrame.height();
			int frameIndex = pointFrame.frame_index();

			const astra::Vector3f* data = pointFrame.data();

			// values without grid gap
			/*const float xGridStep = 1.0 / float(gridSize['x']);
			const float yGridStep = 1.0 / float(gridSize['y']);
			const float zGridStep = 1.0 / float(gridSize['z']);*/

			// values with grid gap
			const float xDelta = CAPTURE_AREA["xMax"] - CAPTURE_AREA["xMin"];
			const float yDelta = CAPTURE_AREA["yMax"] - CAPTURE_AREA["yMin"];
			const float zDelta = CAPTURE_AREA["zMax"] - CAPTURE_AREA["zMin"];

			//cout << "deltas: " << xDelta << " " << yDelta << " " << zDelta << endl;

			const float xNormGap = GRID_GAP / xDelta;
			const float yNormGap = GRID_GAP / yDelta;
			const float zNormGap = GRID_GAP / zDelta;

			//cout << "normGaps: " << xNormGap << " " << yNormGap << " " << zNormGap << endl;
			
			// grid step with no gap
			/*const float xGridStep = 1.0 / float(gridSize['x']);
			const float yGridStep = 1.0 / float(gridSize['y']);
			const float zGridStep = 1.0 / float(gridSize['z']);*/

			// grid step with gap
			const float xGridStep = (1 - (GRID_SIZE['x'] - 1) * xNormGap) / (GRID_SIZE['x']);
			const float yGridStep = (1 - (GRID_SIZE['y'] - 1) * yNormGap) / (GRID_SIZE['y']);
			const float zGridStep = (1 - (GRID_SIZE['z'] - 1) * zNormGap) / (GRID_SIZE['z']);
			
			//cout << "gridSteps: " << xGridStep << " " << yGridStep << " " << zGridStep << endl;

			std::set<std::string> activeGridPositions;
			std::map<std::string, int> activeGridCounts;

			// Init map of point counts with zeros
			/*
			for (int x = 1; x <= GRID_SIZE['x']; x++) {
				for (int y = 1; y <= GRID_SIZE['y']; y++) {
					for (int z = 1; z <= GRID_SIZE['z']; z++) {
						std::ostringstream key;
						key << x << " " << y << " " << z;
						activeGridCounts.insert(pair<string, int>(key.str(), 0));
					}
				}
			}
			*/

			// NOTE: better order for Filipe
			for (int y = 1; y <= GRID_SIZE['y']; y++) {
				for (int x = 1; x <= GRID_SIZE['x']; x++) {
					for (int z = 1; z <= GRID_SIZE['z']; z++) {
						std::ostringstream key;
						key << x << " " << y << " " << z;
						activeGridCounts.insert(pair<string, int>(key.str(), 0));
					}
				}
			}

			for (int i = 0; i < pointFrame.length(); i++)
			{
				// Get point
				const astra::Vector3f* dataPoint = data + i;

				// Crop x
				if (dataPoint->x < CAPTURE_AREA["xMin"] || dataPoint->x > CAPTURE_AREA["xMax"]) {
					//std::cout << "ignoring point because not in capture area (width)" << std::endl;
					continue;
				}

				// Crop y
				if (dataPoint->y < CAPTURE_AREA["yMin"] || dataPoint->y > CAPTURE_AREA["yMax"]) {
					//std::cout << "ignoring point because not in capture area (height)" << std::endl;
					continue;
				}

				// Crop z
				if (dataPoint->z < CAPTURE_AREA["zMin"] || dataPoint->z > CAPTURE_AREA["zMax"]) {
				
					//std::cout << "ignoring point because not in capture area (depth)" << std::endl;
					continue;
				}

				// Normalization
				const float normX = ofMap(float(dataPoint->x), CAPTURE_AREA["xMin"], CAPTURE_AREA["xMax"], 0.0, 1.0, false);
				const float normY = ofMap(float(dataPoint->y), CAPTURE_AREA["yMin"], CAPTURE_AREA["yMax"], 0.0, 1.0, false);
				const float normZ = ofMap(float(dataPoint->z), CAPTURE_AREA["zMin"], CAPTURE_AREA["zMax"], 0.0, 1.0, false);

				//cout << "norms " << normX << " " << normY << " " << normZ << endl;

				const int xGrid = getPositionInGridGap(normX, xGridStep, GRID_SIZE['x'], xNormGap);
				const int yGrid = getPositionInGridGap(normY, yGridStep, GRID_SIZE['y'], yNormGap);
				const int zGrid = getPositionInGridGap(normZ, zGridStep, GRID_SIZE['z'], zNormGap);

				if (xGrid == -1 || yGrid == -1 || zGrid == -1) {
					//cout << "ignore position since in gap" << endl;
					continue;
				}

				std::string gridPosition = "";

				//cout << "positions: " << xGrid << " " << yGrid << " " << zGrid << endl;
				
				// with (x, y, z)
				gridPosition = gridPosition + std::to_string(xGrid) + " " + std::to_string(yGrid) + " " + std::to_string(zGrid);
				
				// with (x, z)
				//gridPosition = gridPosition + "[" + std::to_string(xGrid) + " " + std::to_string(zGrid) + "]";
				//gridPosition = gridPosition + std::to_string(xGrid) + " " + std::to_string(zGrid);
								
				activeGridPositions.insert(gridPosition);
				
				activeGridCounts[gridPosition]++;
				
			}

			// Print all active positions for frame
			set<string>::iterator it;
			ostringstream oss;
			for (it = activeGridPositions.begin(); it != activeGridPositions.end(); ++it) {
				//std::cout << ' ' << *it;
				oss << *it << " ";
			}
			// std::cout << oss.str() << '\n';

			map<string, int>::iterator it2;
			ostringstream oss2;
			ostringstream frameStringStream;
			// Build message to send via OSC
			for (it2 = activeGridCounts.begin(); it2 != activeGridCounts.end(); ++it2) {
				//std::cout << ' ' << *it;
				oss2 << it2->first << " " << it2->second << endl;
				frameStringStream << it2->second << " ";
			}
			// cout << oss2.str() << '\n';
			// cout << "mensagem a enviar: " << frameStringStream.str() << endl;

			const string frameString = frameStringStream.str();
			socket123.send_to(buffer(frameString, frameString.length()), remote_endpoint, 0, err);
			
		}

		/*const int depthWidth = pointFrame.width();
		const int depthHeight = pointFrame.height();*/

		/*init_texture(depthWidth, depthHeight, depthView_);

		if (isPaused_) { return; }

		visualizer_.update(pointFrame);

		astra::RgbPixel* vizBuffer = visualizer_.get_output();
		uint8_t* buffer = &depthView_.buffer_[0];
		for (int i = 0; i < depthWidth * depthHeight; i++)
		{
		const int rgbaOffset = i * 4;
		if (i == 100) cout << buffer[rgbaOffset - 3]  << " " << buffer[rgbaOffset - 2]  << " " << buffer[rgbaOffset - 1]  << endl;
		buffer[rgbaOffset] = vizBuffer[i].r;
		buffer[rgbaOffset + 1] = vizBuffer[i].g;
		buffer[rgbaOffset + 2] = vizBuffer[i].b;
		buffer[rgbaOffset + 3] = 255;
		}*/

		//depthView_.texture_.update(&depthView_.buffer_[0]);
	}

	void update_color(astra::Frame& frame)
	{
		const astra::ColorFrame colorFrame = frame.get<astra::ColorFrame>();

		if (!colorFrame.is_valid())
		{
			clear_view(colorView_);
			colorView_.texture_.update(&colorView_.buffer_[0]);
			return;
		}

		const int colorWidth = colorFrame.width();
		const int colorHeight = colorFrame.height();

		init_texture(colorWidth, colorHeight, colorView_);

		if (isPaused_) { return; }

		const astra::RgbPixel* color = colorFrame.data();
		uint8_t* buffer = &colorView_.buffer_[0];

		for (int i = 0; i < colorWidth * colorHeight; i++)
		{
			const int rgbaOffset = i * 4;
			buffer[rgbaOffset] = color[i].r;
			buffer[rgbaOffset + 1] = color[i].g;
			buffer[rgbaOffset + 2] = color[i].b;
			buffer[rgbaOffset + 3] = 255;
		}

		colorView_.texture_.update(&colorView_.buffer_[0]);
	}

	void update_ir_16(astra::Frame& frame)
	{
		const astra::InfraredFrame16 irFrame = frame.get<astra::InfraredFrame16>();

		if (!irFrame.is_valid())
		{
			clear_view(colorView_);
			colorView_.texture_.update(&colorView_.buffer_[0]);
			return;
		}

		const int irWidth = irFrame.width();
		const int irHeight = irFrame.height();

		init_texture(irWidth, irHeight, colorView_);

		if (isPaused_) { return; }

		const uint16_t* ir_values = irFrame.data();
		uint8_t* buffer = &colorView_.buffer_[0];
		for (int i = 0; i < irWidth * irHeight; i++)
		{
			const int rgbaOffset = i * 4;
			const uint16_t value = ir_values[i];
			const uint8_t red = static_cast<uint8_t>(value >> 2);
			const uint8_t blue = 0x66 - red / 2;
			buffer[rgbaOffset] = red;
			buffer[rgbaOffset + 1] = 0;
			buffer[rgbaOffset + 2] = blue;
			buffer[rgbaOffset + 3] = 255;
		}

		colorView_.texture_.update(&colorView_.buffer_[0]);
	}

	void update_ir_rgb(astra::Frame& frame)
	{
		const astra::InfraredFrameRgb irFrame = frame.get<astra::InfraredFrameRgb>();

		if (!irFrame.is_valid())
		{
			clear_view(colorView_);
			colorView_.texture_.update(&colorView_.buffer_[0]);
			return;
		}

		int irWidth = irFrame.width();
		int irHeight = irFrame.height();

		init_texture(irWidth, irHeight, colorView_);

		if (isPaused_) { return; }

		const astra::RgbPixel* irRGB = irFrame.data();
		uint8_t* buffer = &colorView_.buffer_[0];
		for (int i = 0; i < irWidth * irHeight; i++)
		{
			const int rgbaOffset = i * 4;
			buffer[rgbaOffset] = irRGB[i].r;
			buffer[rgbaOffset + 1] = irRGB[i].g;
			buffer[rgbaOffset + 2] = irRGB[i].b;
			buffer[rgbaOffset + 3] = 255;
		}

		colorView_.texture_.update(&colorView_.buffer_[0]);
	}

	virtual void on_frame_ready(astra::StreamReader& reader,
		astra::Frame& frame) override
	{
		update_depth(frame);

		switch (colorMode_)
		{
		case MODE_COLOR:
			update_color(frame);
			break;
		case MODE_IR_16:
			update_ir_16(frame);
			break;
		case MODE_IR_RGB:
			update_ir_rgb(frame);
			break;
		}

		check_fps();
	}

	void draw_to(sf::RenderWindow& window, sf::Vector2f origin, sf::Vector2f size)
	{
		const int viewSize = (int)(size.x / 2.0f);
		const sf::Vector2f windowSize = window.getView().getSize();

		if (depthView_.buffer_ != nullptr)
		{
			const float depthScale = viewSize / (float)depthView_.width_;
			const int horzCenter = origin.y * windowSize.y;

			depthView_.sprite_.setScale(depthScale, depthScale);
			depthView_.sprite_.setPosition(origin.x * windowSize.x, horzCenter);
			window.draw(depthView_.sprite_);
		}

		if (colorView_.buffer_ != nullptr)
		{
			const float colorScale = viewSize / (float)colorView_.width_;
			const int horzCenter = origin.y * windowSize.y;

			colorView_.sprite_.setScale(colorScale, colorScale);

			if (overlayDepth_)
			{
				colorView_.sprite_.setPosition(origin.x * windowSize.x, horzCenter);
				colorView_.sprite_.setColor(sf::Color(255, 255, 255, 128));
			}
			else
			{
				colorView_.sprite_.setPosition(origin.x * windowSize.x + viewSize, horzCenter);
				colorView_.sprite_.setColor(sf::Color(255, 255, 255, 255));
			}

			window.draw(colorView_.sprite_);
		}
	}

	void toggle_depth_overlay()
	{
		overlayDepth_ = !overlayDepth_;
	}

	bool get_overlay_depth() const
	{
		return overlayDepth_;
	}

	void toggle_paused()
	{
		isPaused_ = !isPaused_;
	}

	bool is_paused() const
	{
		return isPaused_;
	}

	ColorMode get_mode() const { return colorMode_; }
	void set_mode(ColorMode mode) { colorMode_ = mode; }

private:
	samples::common::LitDepthVisualizer visualizer_;

	using DurationType = std::chrono::milliseconds;
	using ClockType = std::chrono::high_resolution_clock;

	ClockType::time_point prev_;
	float elapsedMillis_{ .0f };

	StreamView depthView_;
	StreamView colorView_;
	ColorMode colorMode_;
	bool overlayDepth_{ false };
	bool isPaused_{ false };
};

astra::DepthStream configure_depth(astra::StreamReader& reader)
{
	auto depthStream = reader.stream<astra::DepthStream>();

	auto oldMode = depthStream.mode();

	//We don't have to set the mode to start the stream, but if you want to here is how:
	astra::ImageStreamMode depthMode;

	const int w = 640;
	const int h = 480;
	depthMode.set_width(w);
	depthMode.set_height(h);
	depthMode.set_pixel_format(astra_pixel_formats::ASTRA_PIXEL_FORMAT_DEPTH_MM);
	depthMode.set_fps(30);

	depthStream.set_mode(depthMode);

	auto newMode = depthStream.mode();
	printf("Changed depth mode: %dx%d @ %d -> %dx%d @ %d\n",
		oldMode.width(), oldMode.height(), oldMode.fps(),
		newMode.width(), newMode.height(), newMode.fps());

	return depthStream;
}

astra::InfraredStream configure_ir(astra::StreamReader& reader, bool useRGB)
{
	auto irStream = reader.stream<astra::InfraredStream>();

	auto oldMode = irStream.mode();

	astra::ImageStreamMode irMode;
	const int w = 640;
	const int h = 480;
	irMode.set_width(w);
	irMode.set_height(h);

	if (useRGB)
	{
		irMode.set_pixel_format(astra_pixel_formats::ASTRA_PIXEL_FORMAT_RGB888);
	}
	else
	{
		irMode.set_pixel_format(astra_pixel_formats::ASTRA_PIXEL_FORMAT_GRAY16);
	}

	irMode.set_fps(30);
	irStream.set_mode(irMode);

	auto newMode = irStream.mode();
	printf("Changed IR mode: %dx%d @ %d -> %dx%d @ %d\n",
		oldMode.width(), oldMode.height(), oldMode.fps(),
		newMode.width(), newMode.height(), newMode.fps());

	return irStream;
}

astra::ColorStream configure_color(astra::StreamReader& reader)
{
	auto colorStream = reader.stream<astra::ColorStream>();

	auto oldMode = colorStream.mode();

	astra::ImageStreamMode colorMode;
	colorMode.set_width(640);
	colorMode.set_height(480);
	colorMode.set_pixel_format(astra_pixel_formats::ASTRA_PIXEL_FORMAT_RGB888);
	colorMode.set_fps(30);

	colorStream.set_mode(colorMode);

	auto newMode = colorStream.mode();
	printf("Changed color mode: %dx%d @ %d -> %dx%d @ %d\n",
		oldMode.width(), oldMode.height(), oldMode.fps(),
		newMode.width(), newMode.height(), newMode.fps());

	return colorStream;
}

int main(int argc, char** argv)
{

	// NOTE: snippet to determin C++ standard being used
	/*if (__cplusplus == 201703L) std::cout << "C++17\n";
	else if (__cplusplus == 201402L) std::cout << "C++14\n";
	else if (__cplusplus == 201103L) std::cout << "C++11\n";
	else if (__cplusplus == 199711L) std::cout << "C++98\n";
	else std::cout << "pre-standard C++\n";*/

	/*system("pause");
	return 0;*/

    // send socket
	socket123.open(ip::udp::v4());
	remote_endpoint = ip::udp::endpoint(ip::address::from_string(SEND_IP), SEND_PORT);
	//remote_endpoint = ip::udp::endpoint(ip::address::from_string("169.254.6.109"), 9000);

	// receive socket
	socket_receive.open(ip::udp::v4());
	receive_endpoint = ip::udp::endpoint(ip::address::from_string(SEND_IP), RECEIVE_PORT);
	
	// istringstream frameStringStream;
	// string frameString = frameStringStream.str();
	// socket_receive.async_receive_from(buffer(frameString), 0, receive_endpoint, handler_osc_receive);
	// socket_receive.bind(ip::udp::endpoint(ip::address::from_string(SEND_IP), RECEIVE_PORT));
	// socket_receive.async_receive_from(buffer(recv_buf), receive_endpoint, handler_osc_receive);
	

	// TEST: getPositionInGridGap
	//const unsigned int gridSize = 4;
	//const float gridGap = 0.05; // 5%
	//const float gridStep = (1 - (gridSize - 1) * gridGap) / gridSize;
	//cout << (0.0, gridStep, gridSize, gridGap) << endl; // 1
	//cout << getPositionInGridGap(0.2, gridStep, gridSize, gridGap) << endl; // 1
	//cout << getPositionInGridGap(0.25, gridStep, gridSize, gridGap) << endl; // -1 (gap)
	//cout << getPositionInGridGap(0.26, gridStep, gridSize, gridGap) << endl; // -1 (gap)
	//cout << getPositionInGridGap(0.3, gridStep, gridSize, gridGap) << endl; // 2
	//cout << getPositionInGridGap(0.9, gridStep, gridSize, gridGap) << endl; // 4
	//cout << getPositionInGridGap(1, gridStep, gridSize, gridGap) << endl; // 4
	
	/*system("pause");
	return 0;*/

	Client client;
    std::thread r([&] { client.Receiver(); });

    // std::string input = argc>1? argv[1] : "hello world";
    // std::cout << "Input is '" << input.c_str() << "'\nSending it to Sender Function...\n";

    // for (int i = 0; i < 3; ++i) {
    //     std::this_thread::sleep_for(std::chrono::milliseconds(200));
    //     Sender(input);
    // }

    // r.join();
	
	astra::initialize();

	set_key_handler();

#ifdef _WIN32
	auto fullscreenStyle = sf::Style::None;
#else
	auto fullscreenStyle = sf::Style::Fullscreen;
#endif

	const sf::VideoMode fullScreenMode = sf::VideoMode::getFullscreenModes()[0];
	const sf::VideoMode windowedMode(1800, 675);

	bool isFullScreen = false;
	sf::RenderWindow window(windowedMode, "Stream Viewer");

	astra::StreamSet streamSet;
	astra::StreamReader reader = streamSet.create_reader();

	reader.stream<astra::PointStream>().start();

	auto depthStream = configure_depth(reader);
	depthStream.start();

	auto colorStream = configure_color(reader);
	colorStream.start();

	auto irStream = configure_ir(reader, false);

	MultiFrameListener listener;
	listener.set_mode(MODE_COLOR);

	reader.add_listener(listener);

	while (window.isOpen())
	{
		astra_update();

		sf::Event event;
		while (window.pollEvent(event))
		{
			switch (event.type)
			{
			case sf::Event::Closed:
				window.close();
				break;
			case sf::Event::KeyPressed:
			{
				switch (event.key.code)
				{
				case sf::Keyboard::Escape:
					window.close();
					break;
				case sf::Keyboard::F:
					if (isFullScreen)
					{
						isFullScreen = false;
						window.create(windowedMode, "Stream Viewer", sf::Style::Default);
					}
					else
					{
						isFullScreen = true;
						window.create(fullScreenMode, "Stream Viewer", fullscreenStyle);
					}
					break;
				case sf::Keyboard::R:
					depthStream.enable_registration(!depthStream.registration_enabled());
					break;
				case sf::Keyboard::M:
				{
					const bool newMirroring = !depthStream.mirroring_enabled();
					depthStream.enable_mirroring(newMirroring);
					colorStream.enable_mirroring(newMirroring);
					irStream.enable_mirroring(newMirroring);
				}
				break;
				case sf::Keyboard::G:
					colorStream.stop();
					configure_ir(reader, false);
					listener.set_mode(MODE_IR_16);
					irStream.start();
					break;
				case sf::Keyboard::I:
					colorStream.stop();
					configure_ir(reader, true);
					listener.set_mode(MODE_IR_RGB);
					irStream.start();
					break;
				case sf::Keyboard::O:
					listener.toggle_depth_overlay();
					if (listener.get_overlay_depth())
					{
						depthStream.enable_registration(true);
					}
					break;
				case sf::Keyboard::P:
					listener.toggle_paused();
					break;
				case sf::Keyboard::C:
					if (event.key.control)
					{
						window.close();
					}
					else
					{
						irStream.stop();
						listener.set_mode(MODE_COLOR);
						colorStream.start();
					}
					break;
				default:
					break;
				}
				break;
			}
			default:
				break;
			}
		}

		// clear the window with black color
		window.clear(sf::Color::Black);
		listener.draw_to(window, sf::Vector2f(0.f, 0.f), sf::Vector2f(window.getSize().x, window.getSize().y));
		window.display();

		if (!shouldContinue)
		{
			window.close();
		}
	}

	astra::terminate();
	return 0;
}