#include "cinder/app/AppNative.h"
#include "cinder/gl/gl.h"
#include "cinder/gl/Light.h"
#include "cinder/Utilities.h"
#include "cinder/MayaCamUI.h"
#include "AntTweakBar.h"
#include "mndlkit/params/PParams.h"
#include "BulletWorld.h"
#include "Cinder-LeapSdk.h"

using namespace ci;
using namespace ci::app;
using namespace std;
using namespace LeapSdk;


class BulletAssimpModelApp : public AppNative
{
	typedef std::vector< int > Gestures;

public:
	void setup();
	void mouseDown( MouseEvent event );
	void mouseDrag( MouseEvent event );
	void mouseUp( MouseEvent event );
	void keyDown( KeyEvent event );
	void update();
	void draw();
	void resize();
	void shutdown();

protected:
	void setupParams();

	int  getActHand( Frame& frame, int actHand );

protected:
	BulletWorld mBulletWorld;
	AssimpModel *mAssimpModel;
	AssimpModel *mAssimpModelDebug;

	mndl::params::PInterfaceGl mParams;
	float mFps;

	// camera
	MayaCamUI mMayaCam;
	bool      mCameraLock;
	float     mCameraFov;
	Vec3f     mCameraEyePoint;
	Vec3f     mCameraCenterOfInterestPoint;
	static const int mStepKey = 3;

	// assimp model
	Vec3f     mPosition;
	Vec3f     mDirection;
	Vec3f     mNormal;

	// Leap
	uint32_t                mCallbackId;
	LeapSdk::HandMap        mHands;
	LeapSdk::DeviceRef      mLeap;
	void                    onFrame( LeapSdk::Frame frame );
	int                     mActHand;
	Gestures                mActGestures;
	Vec3f                   mHandPos;
	Vec3f                   mHandDir;
	Vec3f                   mHandNorm;
	bool                    mDrawVectors;

	gl::Light* mLight;
};

void BulletAssimpModelApp::setup()
{
	mAssimpModel = 0;
	mAssimpModelDebug = 0;

	gl::enableDepthRead();
	gl::enableDepthWrite();

	setupParams();

	CameraPersp cam;
	cam.setPerspective( mCameraFov, getWindowAspectRatio(), 0.1f, 1000.0f );
	cam.setEyePoint( mCameraEyePoint );
	cam.setCenterOfInterestPoint( mCameraCenterOfInterestPoint );
	mMayaCam.setCurrentCam( cam );

	{
		glEnable( GL_LIGHTING );
		glEnable( GL_DEPTH_TEST );
		glEnable( GL_RESCALE_NORMAL );

		//create light
		mLight = new gl::Light( gl::Light::DIRECTIONAL, 0 );
		mLight->lookAt( Vec3f( 0, 30, 0 ), Vec3f( 0, 0, 0 ) );
		mLight->setAmbient( Color( 1.0f, 1.0f, 1.0f ) );
		mLight->setDiffuse( Color( 1.0f, 1.0f, 1.0f ) );
		mLight->setSpecular( Color( 1.0f, 1.0f, 1.0f ) );
		mLight->setShadowParams( 100.0f, 1.0f, 20.0f );
//		mLight->update( cam );
//		mLight->enable();
	}

	mBulletWorld.setup();

//	AssimpModel assimpModel();

	// Start device
	mLeap = Device::create();
	mCallbackId = mLeap->addCallback( &BulletAssimpModelApp::onFrame, this );
	mLeap->enableGesture( Leap::Gesture::TYPE_KEY_TAP );
	mActHand = -1;
	mActGestures.clear();
}

// Called when Leap frame data is ready
void BulletAssimpModelApp::onFrame( Frame frame )
{
	mHands = frame.getHands();

	mActHand = getActHand( frame, mActHand );
	mActGestures.clear();

	if( mActHand != -1 )
	{
		const vector<Leap::Gesture>& gestures = frame.getGestures();
		for( vector<Leap::Gesture>::const_iterator iter = gestures.begin(); iter != gestures.end(); ++iter )
		{
			Gesture::Type type = iter->type();

			if( type == Leap::Gesture::TYPE_KEY_TAP )
			{
// 				Gesture::State state = iter->state();
// 
// 				if( state == Gesture::State::STATE_START )
				{
					const Leap::KeyTapGesture& gesture = (Leap::KeyTapGesture)*iter;
					int actFinger = 0;
					const LeapSdk::FingerMap& fingers = mHands[ mActHand ].getFingers();
					for( FingerMap::const_iterator fingerIter = fingers.begin(); fingerIter != fingers.end(); ++fingerIter )
					{
						const Finger& finger = fingerIter->second;

						if( gesture.pointable().id() == finger.getId() )
						{
							mActGestures.push_back( actFinger - 1 );
						}

						++actFinger;
					}
				}
			}
		}
	}
}

int BulletAssimpModelApp::getActHand( Frame& frame, int actHand )
{
	if( mHands.size())
	{
		if( actHand == -1 )
		{
			for( HandMap::const_iterator it = mHands.begin(); it != mHands.end(); ++it )
			{
				Hand hand = it->second;

				if( hand.getFingers().size() == 5 )
				{
					return it->first;
				}
			}
		}
		else
		{
			HandMap::const_iterator it = mHands.find( mActHand );

			if( it != mHands.end())
			{
				return mHands.begin()->first;
			}
		}
	}

	return -1;
}

void BulletAssimpModelApp::setupParams()
{
	mndl::params::PInterfaceGl::load( "params.xml" );

	mParams = mndl::params::PInterfaceGl( "Parameters", Vec2i( 230, 550 ), Vec2i( 50, 50 ) );
	mParams.addPersistentSizeAndPosition();

	mFps = 0;
	mParams.addParam( "Fps", &mFps, "", true );
	mParams.addSeparator();
	mParams.addText( "Camera" );
	mParams.addPersistentParam( "Lock camera (l)", &mCameraLock, false );
	mParams.addPersistentParam( "Fov", &mCameraFov, 45.f, "min=20 max=180 step=.1" );
	mParams.addPersistentParam( "Eye", &mCameraEyePoint, Vec3f( 0.0f, 10.0f, -40.0f ));
	mParams.addPersistentParam( "Center of Interest", &mCameraCenterOfInterestPoint, Vec3f( 0.0f, 10.0f, 0.0f ));
	mParams.addText( "Assimp test" );
	mPosition = Vec3f( 0.0f, 10.0f, 0.0f );
//	mParams.addPersistentParam( "Position" , &mPosition , Vec3f( 0.0f, 10.0f, 0.0f ));
	mParams.addPersistentParam( "Direction", &mDirection, Vec3f( 0.0f,  0.0f, -1.0f ));
	mParams.addPersistentParam( "Normal"   , &mNormal   , Vec3f( 0.0f, -1.0f, 0.0f ));

	mParams.addButton( "Spawn (k)", [ this ]()
								{
									if( mAssimpModelDebug )
										mBulletWorld.removeAssimpModel( mAssimpModelDebug );
									mAssimpModelDebug = mBulletWorld.spawnAssimpModel( mPosition ); //* 10 );
								} );

	mParams.addButton( "Sing (0)", [ this ]()
								{
									if( mAssimpModelDebug )
										mAssimpModelDebug->doAnimate( 0 );
								} );

	mParams.addButton( "Fly (1)", [ this ]()
								{
									if( mAssimpModelDebug )
										mAssimpModelDebug->doAnimate( 1 );
								} );

	mParams.addButton( "Wag (2)", [ this ]()
								{
									if( mAssimpModelDebug )
										mAssimpModelDebug->doAnimate( 2 );
								} );

	mParams.addText( "Leap" );
	mParams.addPersistentParam( "Hand pos" , &mHandPos , Vec3f( -1, -1, -1 ), "", true );
	mParams.addPersistentParam( "Hand dir" , &mHandDir , Vec3f( -1, -1, -1 ), "", true );
	mParams.addPersistentParam( "Hand norm", &mHandNorm, Vec3f( -1, -1, -1 ), "", true );
	mParams.addPersistentParam( "DrawVectors", &mDrawVectors, false );
}

void BulletAssimpModelApp::mouseDown( MouseEvent event )
{
	if ( mCameraLock )
		mBulletWorld.mouseDown( event, mMayaCam.getCamera());
	else
		mMayaCam.mouseDown( event.getPos() );
}

void BulletAssimpModelApp::mouseDrag( MouseEvent event )
{
	if ( mCameraLock )
		mBulletWorld.mouseDrag( event, mMayaCam.getCamera());
	else
		mMayaCam.mouseDrag( event.getPos(), event.isLeftDown(), event.isMiddleDown(), event.isRightDown() );
}

void BulletAssimpModelApp::mouseUp( MouseEvent event )
{
	if ( mCameraLock )
		mBulletWorld.mouseUp( event, mMayaCam.getCamera());
}

void BulletAssimpModelApp::keyDown( KeyEvent event )
{
	switch( event.getCode() )
	{
	case KeyEvent::KEY_f:
		if ( ! isFullScreen() )
		{
			setFullScreen( true );
			if ( mParams.isVisible() )
				showCursor();
			else
				hideCursor();
		}
		else
		{
			setFullScreen( false );
			showCursor();
		}
		break;

	case KeyEvent::KEY_s:
		{
			mndl::params::PInterfaceGl::showAllParams( !mParams.isVisible() );
			if ( isFullScreen() )
			{
				if ( mParams.isVisible() )
					showCursor();
				else
					hideCursor();
			}
			break;
		}

	case KeyEvent::KEY_l:
		{
			mCameraLock = ! mCameraLock;
		}
		break;
	case KeyEvent::KEY_k:
		{
			if( mAssimpModelDebug )
				mBulletWorld.removeAssimpModel( mAssimpModelDebug );
			mAssimpModelDebug = mBulletWorld.spawnAssimpModel( mPosition ); //* 10 );
		}
		break;
	case KeyEvent::KEY_LEFT:
		{
			mMayaCam.mouseDown( Vec2i( mStepKey, 0 ));
			mMayaCam.mouseDrag( Vec2i( mStepKey, 0 ), true, false, false );
			mMayaCam.mouseDrag( Vec2i( 0       , 0 ), true, false, false );
		}
		break;
	case KeyEvent::KEY_RIGHT:
		{
			mMayaCam.mouseDown( Vec2i( 0       , 0 ));
			mMayaCam.mouseDrag( Vec2i( 0       , 0 ), true, false, false );
			mMayaCam.mouseDrag( Vec2i( mStepKey, 0 ), true, false, false );
		}
		break;
	case KeyEvent::KEY_UP:
		{
			mMayaCam.mouseDown( Vec2i( 0, mStepKey ));
			mMayaCam.mouseDrag( Vec2i( 0, mStepKey ), true, false, false );
			mMayaCam.mouseDrag( Vec2i( 0, 0        ), true, false, false );
		}
		break;
	case KeyEvent::KEY_DOWN:
		{
			mMayaCam.mouseDown( Vec2i( 0, 0        ));
			mMayaCam.mouseDrag( Vec2i( 0, 0        ), true, false, false );
			mMayaCam.mouseDrag( Vec2i( 0, mStepKey ), true, false, false );
		}
		break;
	case KeyEvent::KEY_ESCAPE:
		quit();
		break;
	case KeyEvent::KEY_0:
	case KeyEvent::KEY_1:
	case KeyEvent::KEY_2:
	case KeyEvent::KEY_3:
	case KeyEvent::KEY_4:
	case KeyEvent::KEY_5:
	case KeyEvent::KEY_6:
	case KeyEvent::KEY_7:
	case KeyEvent::KEY_8:
	case KeyEvent::KEY_9:
		{
			int pos = event.getCode() - KeyEvent::KEY_0;

			if( mAssimpModelDebug )
				mAssimpModelDebug->doAnimate( pos );
			if( mAssimpModel )
				mAssimpModel->doAnimate( pos );
		}
		break;
	default:
		mBulletWorld.keyDown( event );
	}
}

void BulletAssimpModelApp::update()
{
	mFps = getAverageFps();

	mBulletWorld.update();

	CameraPersp cam = mMayaCam.getCamera();
	if ( cam.getFov() != mCameraFov )
	{
		cam.setPerspective( mCameraFov, getWindowAspectRatio(), 0.1f, 1000.0f );
		mMayaCam.setCurrentCam( cam );
	}
	if( mCameraLock )
	{
		if( mCameraEyePoint != cam.getEyePoint())
		{
			cam.setEyePoint( mCameraEyePoint );
			mMayaCam.setCurrentCam( cam );
		}
		if( mCameraCenterOfInterestPoint != cam.getCenterOfInterestPoint())
		{
			cam.setCenterOfInterestPoint( mCameraCenterOfInterestPoint );
			mMayaCam.setCurrentCam( cam );
		}
	}
	else
	{
		mCameraEyePoint              = cam.getEyePoint();
		mCameraCenterOfInterestPoint = cam.getCenterOfInterestPoint();
	}

	mLight->update( cam );
	mLight->enable();

	// Update device
	if( mLeap && mLeap->isConnected() )
	{
		mLeap->update();
	}

	if( mActHand != -1 )
	{
		Hand hand = mHands[ mActHand ];

		mHandPos  = hand.getPosition();
		mHandDir  = hand.getDirection();
		mHandNorm = hand.getNormal();

		if( ! mAssimpModel )
			mAssimpModel = mBulletWorld.spawnAssimpModel( mHandPos );

		mBulletWorld.updateAssimpModel( mAssimpModel, mHandPos, mHandDir, mHandNorm );
	}
	else
	{
		if( mAssimpModel )
		{
			mBulletWorld.removeAssimpModel( mAssimpModel );
			mAssimpModel = 0;
		}

		mHandPos  = Vec3f( -1, -1, -1 );
		mHandDir  = Vec3f( -1, -1, -1 );
		mHandNorm = Vec3f( -1, -1, -1 );
	}

	if( mAssimpModelDebug )
		mBulletWorld.updateAssimpModel( mAssimpModelDebug, mPosition, mDirection.normalized(), mNormal.normalized() );

	for( Gestures::const_iterator it = mActGestures.begin(); it != mActGestures.end(); ++it )
	{
		int animPos = *it;
		if( mAssimpModelDebug )
			mAssimpModelDebug->doAnimate( animPos );
		if( mAssimpModel )
			mAssimpModel->doAnimate( animPos );
	}
}

void BulletAssimpModelApp::draw()
{
	// clear out the window with black
	gl::clear( Colorf( 0.392, 0.392, 0.784 ));

	gl::setViewport( getWindowBounds() );
	gl::setMatrices( mMayaCam.getCamera() );

	gl::enableDepthRead();
	gl::enableDepthWrite();

	mBulletWorld.draw();
	mndl::params::PInterfaceGl::draw();

	if( mDrawVectors && ( mAssimpModel || mAssimpModelDebug ) )
	{
		Quatf rot = Quatf( Vec3f::yAxis(), M_PI / 2.0f );

		Vec3f handDir;
		Vec3f handNorm;

		if( mAssimpModel )
		{
			handDir  = rot * mHandDir;
			handNorm = rot * mHandNorm;
		}
		else if( mAssimpModelDebug )
		{
			handDir  = rot * mDirection;
			handNorm = rot * mNormal;
		}

		glColor4ub( 255, 0, 0, 255 );
		gl::drawVector( Vec3f::zero(), handDir.normalized(), .5, .2 );
		glColor4ub( 0, 255, 0, 255 );
		gl::drawVector( Vec3f::zero(), handNorm.normalized(), .5, .2 );
	}
}

void BulletAssimpModelApp::resize()
{
	CameraPersp cam = mMayaCam.getCamera();
	cam.setAspectRatio( getWindowAspectRatio() );
	mMayaCam.setCurrentCam( cam );
}

void BulletAssimpModelApp::shutdown()
{
	mLeap->removeCallback( mCallbackId );
	mHands.clear();

	mndl::params::PInterfaceGl::save();
}

CINDER_APP_NATIVE( BulletAssimpModelApp, RendererGl )
