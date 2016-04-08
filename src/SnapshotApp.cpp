#include "cinder/app/App.h"
#include "cinder/app/RendererGl.h"
#include "cinder/gl/gl.h"
#include "Kinect2.h"
#include <fstream>
#include <iostream>

using namespace ci;
using namespace ci::app;
using namespace std;

class SnapshotApp : public App {
  public:
	  SnapshotApp();

	void setup() override;
	void keyDown( KeyEvent event ) override;
	void update() override;
	void draw() override;
	void shutdown();
  private:
	  Kinect2::BodyFrame mBodyFrame;
	  ci::Channel8uRef mChannelBodyIndex;
	  ci::Channel16uRef mChannelDepth;
	  Kinect2::DeviceRef mDevice;

	  ofstream myfile;
	  boolean snapshot;
};

SnapshotApp::SnapshotApp() {
	mDevice = Kinect2::Device::create();
	mDevice->start();
	mDevice->connectBodyEventHandler([&](const Kinect2::BodyFrame frame) {
		mBodyFrame = frame;
	});
	mDevice->connectBodyIndexEventHandler([&](const Kinect2::BodyIndexFrame frame)
	{
		mChannelBodyIndex = frame.getChannel();
	});
	mDevice->connectDepthEventHandler([&](const Kinect2::DepthFrame frame)
	{
		mChannelDepth = frame.getChannel();
	});
}

void SnapshotApp::setup()
{
	snapshot = false;
	// create csv file
	myfile.open("skeletal_tracking.csv");
	myfile << "Kat's Test\n";
	vector<std::string> joints = { "SpineBase", "SpineMid", "Neck", "Head", "ShoulderLeft", "ElbowLeft", "WristLeft", "HandLeft",
		"ShoulderRight", "ElbowRight", "WristRight", "HandRight", "HipLeft", "KneeLeft", "AnkleLeft", "FootLeft", "HipRight", "KneeRight",
		"AnkleRight", "FootRight", "SpineShoulder", "HandTipLeft", "ThumbLeft", "HandTipRight", "ThumbRight" };
	for (std::string s : joints) {
		myfile << s + ".x,";
		myfile << s + ".y,";
		myfile << s + ".z,";
	}
	myfile << "\n";
}

void SnapshotApp::keyDown( KeyEvent event )
{
	char key = event.getChar();
	if (key == 'a') {
		snapshot = !snapshot;
	}
	else if (key == '1') {
		myfile << "ANXIETY\n";
	}
	else if (key == '2') {
		myfile << "DISINTEREST\n";
	}
	else if (key == '3') {
		myfile << "INTEREST\n";
	}
	else if (key == '4') {
		myfile << "JOY\n";
	}
	else if (key == '5') {
		myfile << "TIRED\n";
	}
	else if (key == '6') {
		myfile << "ANGER\n";
	}
	else if (key == '7') {
		myfile << "CONFIDENT\n";
	}
	else if (key == '8') {
		myfile << "VUNERABLE\n";
	}
}

void SnapshotApp::update()
{
}

void SnapshotApp::draw()
{
	const gl::ScopedViewport scopedViewport(ivec2(0), getWindowSize());
	const gl::ScopedMatrices scopedMatrices;
	const gl::ScopedBlendAlpha scopedBlendAlpha;
	gl::setMatricesWindow(getWindowSize());
	gl::clear();
	gl::color(ColorAf::white());
	gl::disableDepthRead();
	gl::disableDepthWrite();

	if (mChannelDepth) {
		gl::enable(GL_TEXTURE_2D);
		const gl::TextureRef tex = gl::Texture::create(*Kinect2::channel16To8(mChannelDepth));
		gl::draw(tex, tex->getBounds(), Rectf(getWindowBounds()));
	}

	if (mChannelBodyIndex) {
		gl::enable(GL_TEXTURE_2D);

		auto drawHand = [&](const Kinect2::Body::Hand& hand, const ivec2& pos) -> void
		{
			switch (hand.getState()) {
			case HandState_Closed:
				gl::color(ColorAf(1.0f, 0.0f, 0.0f, 0.5f));
				break;
			case HandState_Lasso:
				gl::color(ColorAf(0.0f, 0.0f, 1.0f, 0.5f));
				break;
			case HandState_Open:
				gl::color(ColorAf(0.0f, 1.0f, 0.0f, 0.5f));
				break;
			default:
				gl::color(ColorAf(0.0f, 0.0f, 0.0f, 0.0f));
				break;
			}
			gl::drawSolidCircle(pos, 30.0f, 32);
		};

		gl::pushMatrices();
		gl::scale(vec2(getWindowSize()) / vec2(mChannelBodyIndex->getSize()));
		gl::disable(GL_TEXTURE_2D);
		for (const Kinect2::Body &body : mBodyFrame.getBodies()) {
			if (body.isTracked()) {
				gl::color(ColorAf::white());
				for (const auto& joint : body.getJointMap()) {
					console() << joint.first << endl;
					if (joint.second.getTrackingState() == TrackingState::TrackingState_Tracked) {
						vec2 pos(mDevice->mapCameraToDepth(joint.second.getPosition()));
						if (snapshot) {
							myfile << to_string(joint.first) + " " + to_string(joint.second.getPosition().x) + ",";
							myfile << to_string(joint.second.getPosition().y) + ",";
							myfile << to_string(joint.second.getPosition().z) + ",";
						}
						gl::drawSolidCircle(pos, 5.0f, 32);
						vec2 parent(mDevice->mapCameraToDepth(body.getJointMap().at(joint.second.getParentJoint()).getPosition()));
						gl::drawLine(pos, parent);
					}
					else {
						myfile << "N/A,N/A,N/A,";
					}
				}
				myfile << "\n";
				drawHand(body.getHandLeft(), mDevice->mapCameraToDepth(body.getJointMap().at(JointType_HandLeft).getPosition()));
				drawHand(body.getHandRight(), mDevice->mapCameraToDepth(body.getJointMap().at(JointType_HandRight).getPosition()));
			}
		}
	}
}

void SnapshotApp::shutdown() {
	mDevice->stop();
	myfile.close();
}

CINDER_APP( SnapshotApp, RendererGl )
