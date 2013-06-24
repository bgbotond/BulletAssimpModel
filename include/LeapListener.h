#pragma once

#include <boost/signals2/signal.hpp>
#include <mutex>

#include "Cinder-Leap.h"

namespace mndl { namespace leap {

class LeapListener : public Leap::Listener
{
	public:
		virtual void onConnect( const Leap::Controller &controller );
		virtual void onFrame( const Leap::Controller &controller );

		const Leap::HandList & getHands() const;
		const Leap::InteractionBox & getInteractionBox() const;


		typedef void( TapCallback )( int32_t );
		typedef boost::signals2::signal< TapCallback > TapSignal;

		template< typename T >
		boost::signals2::connection connectTap( void( T::*fn )( int32_t ), T *obj )
		{
			return mTapSignal.connect( std::function< TapCallback >( std::bind( fn, obj, std::_1 ) ) );
		}

	protected:
		Leap::HandList mHandList;
		Leap::InteractionBox mInteractionBox;
		mutable std::mutex mMutex;

		TapSignal mTapSignal;
};

} } // mndl::leap

