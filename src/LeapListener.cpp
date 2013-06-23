#include "cinder/app/App.h"

#include "LeapListener.h"

using namespace ci;

namespace mndl { namespace leap {

void LeapListener::onFrame( const Leap::Controller &controller )
{
	std::lock_guard< std::mutex > lock( mMutex );
	const Leap::Frame frame = controller.frame();
	mHandList = frame.hands();
	mInteractionBox = frame.interactionBox();
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

