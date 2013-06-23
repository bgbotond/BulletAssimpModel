#include "cinder/app/App.h"

#include "LeapListener.h"

using namespace ci;

namespace mndl { namespace leap {

void LeapListener::onConnect( const Leap::Controller &controller )
{
	controller.enableGesture( Leap::Gesture::TYPE_KEY_TAP );
}

void LeapListener::onFrame( const Leap::Controller &controller )
{
	std::lock_guard< std::mutex > lock( mMutex );
	const Leap::Frame frame = controller.frame();
	mHandList = frame.hands();
	mInteractionBox = frame.interactionBox();

	// Get gestures
	const Leap::GestureList gestures = frame.gestures();
	for ( int g = 0; g < gestures.count(); ++g )
	{
		Leap::Gesture gesture = gestures[ g ];

		if ( gesture.type() == Leap::Gesture::TYPE_KEY_TAP )
		{
			Leap::PointableList gesturePointables = gesture.pointables();
			Leap::HandList gestureHands = gesture.hands();
		}
	}
}

const Leap::HandList & LeapListener::getHands() const
{
	std::lock_guard< std::mutex > lock( mMutex );
	return mHandList;
}

const Leap::InteractionBox & LeapListener::getInteractionBox() const
{
	std::lock_guard< std::mutex > lock( mMutex );
	return mInteractionBox;
}

} } // mndl::leap

