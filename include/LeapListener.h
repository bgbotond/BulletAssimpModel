#pragma once

#include <mutex>

#include "Cinder-Leap.h"

namespace mndl { namespace leap {

class LeapListener : public Leap::Listener
{
	public:
		virtual void onFrame( const Leap::Controller &controller );

		const Leap::HandList & getHands() const;
		const Leap::InteractionBox & getInteractionBox() const;

	protected:
		Leap::HandList mHandList;
		Leap::InteractionBox mInteractionBox;
		mutable std::mutex mMutex;
};

} } // mndl::leap

