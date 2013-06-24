#include "cinder/app/AppNative.h"
#include "cinder/gl/gl.h"
#include "cinder/gl/Light.h"
#include "cinder/gl/Texture.h"
#include "cinder/ImageIo.h"
#include "cinder/MayaCamUI.h"
#include "cinder/Rand.h"
#include "cinder/Timeline.h"
#include "cinder/Utilities.h"

#include "mndlkit/params/PParams.h"
#include "Cinder-Leap.h"

#include "BulletWorld.h"
#include "GlobalData.h"
#include "LeapListener.h"

using namespace ci;
using namespace ci::app;
using namespace std;

class BulletAssimpModelApp : public AppNative
{
public:
	void prepareSettings( Settings *settings );
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

	//int  getActHand( Frame& frame, int actHand );

	BulletWorld mBulletWorld;
	AssimpModel *mAssimpModel;

	mndl::params::PInterfaceGl mParams;
	float mFps;

	// camera
	MayaCamUI mMayaCam;
	bool      mCameraLock;
	float     mCameraFov;
	Vec3f     mCameraEyePoint;
	Vec3f     mCameraCenterOfInterestPoint;
	static const int mStepKey = 3;

	// Leap
	void updateLeap();

	Leap::Controller mLeapController;
	mndl::leap::LeapListener mLeapListener;

	Vec3f mHandPos;
	Vec3f mHandDir;
	Vec3f mHandNorm;
	float mSmoothing;

	bool mDrawVectors;
	Vec3f mMovementRange;
	float mParallaxScale;

	gl::Light* mLight;
	Vec3f      mLightDirection;

	// stage
	void loadBackgroundLayers( const fs::path &relativeDir );
	struct LayerInfo
	{
		string mName;
		Vec3f mOrigPos;
	};
	typedef std::shared_ptr< LayerInfo > LayerInfoRef;

	struct ModelInfo
	{
		mndl::assimp::AssimpLoaderRef mModel;
		Anim< double > mTimer; // start/end animation

		float mMinLayerDepth, mMaxLayerDepth; // minimum, maximum layer node depth (y-coordinate)
		vector< LayerInfoRef > mLayerNodes; // nodes in the model with meshes
	};
	typedef std::shared_ptr< ModelInfo > ModelInfoRef;
	vector< ModelInfoRef > mBackgroundLayers;

	int mCameraIndex;
	vector< CameraPersp > mCameras;

	enum
	{
		STATE_IDLE = 0,
		STATE_GAME
	};
	int mState;

	void startGame();
	void endGame();

	gl::Texture mIconLayer;
	Anim< float > mIconAlpha;

	void onTap( int32_t id );
};

void BulletAssimpModelApp::prepareSettings( Settings *settings )
{
	settings->setWindowSize( 1280, 800 );
}

void BulletAssimpModelApp::setup()
{
	mState = STATE_IDLE;
	mAssimpModel = NULL;

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
		mLight->setAmbient( Color( 1.0f, 1.0f, 1.0f ) );
		mLight->setDiffuse( Color( 1.0f, 1.0f, 1.0f ) );
		mLight->setSpecular( Color( 1.0f, 1.0f, 1.0f ) );
		mLight->setShadowParams( 100.0f, 1.0f, 20.0f );
//		mLight->update( cam );
//		mLight->enable();
	}

	mBulletWorld.setup();

	loadBackgroundLayers( "stage" );

	mIconLayer = loadImage( loadAsset( "gui/icons.png" ) );
	mIconAlpha = 0.f;

	// Leap
	mLeapController.addListener( mLeapListener );
	mLeapListener.connectTap< BulletAssimpModelApp >( &BulletAssimpModelApp::onTap, this );

	// sounds
	GlobalData::get().mAudio.setup( getAssetPath( "sfx" ) );
}

void BulletAssimpModelApp::loadBackgroundLayers( const fs::path &relativeDir )
{
	fs::path dataPath = app::getAssetPath( relativeDir );

	if ( dataPath.empty() )
	{
		app::console() << "Could not find model directory assets/" << relativeDir.string() << std::endl;
		return;
	}

	vector< string > cameraNames;

	// default camera
	CameraPersp cam;
	cam.setPerspective( 60, getWindowAspectRatio(), 0.1f, 1000.0f );
	cam.setEyePoint( Vec3f( 0, 7, 20 ) );
	cam.setCenterOfInterestPoint( Vec3f( 0, 7, 0 ) );
	cameraNames.push_back( "default" );
	mCameras.push_back( cam );

	// load models
	for ( fs::directory_iterator it( dataPath ); it != fs::directory_iterator(); ++it )
	{
		if ( fs::is_regular_file( *it ) &&
				( ( it->path().extension().string() == ".dae" ) ||
				  ( it->path().extension().string() == ".obj" ) ) )
		{
			mndl::assimp::AssimpLoaderRef model;

			try
			{
				model = mndl::assimp::AssimpLoader::create( getAssetPath( relativeDir / it->path().filename() ) );
			}
			catch ( const mndl::assimp::AssimpLoaderExc &exc  )
			{
				app::console() << "Unable to load model " << it->path() << ": " << exc.what() << std::endl;
			}
			if ( model )
			{
				ModelInfoRef mi = ModelInfoRef( new ModelInfo() );
				model->setAnimation( 0 );
				model->enableTextures( true );
				model->enableSkinning( false );
				model->enableAnimation( true );
				mi->mModel = model;
				mi->mTimer = 0.;

				float minDepth = numeric_limits< float >::max();
				float maxDepth = numeric_limits< float >::min();

				const vector< string > &nodeNames = model->getNodeNames();
				for ( auto it = nodeNames.begin(); it != nodeNames.end(); ++it )
				{
					const mndl::assimp::AssimpNodeRef node = model->getAssimpNode( *it );
					if ( node->mMeshes.empty() )
						continue;

					LayerInfoRef layer = LayerInfoRef( new LayerInfo() );
					layer->mName = *it;
					layer->mOrigPos = node->getPosition();
					if ( layer->mOrigPos.y < minDepth )
						minDepth = layer->mOrigPos.y;
					if ( layer->mOrigPos.y > maxDepth )
						maxDepth = layer->mOrigPos.y;

					mi->mLayerNodes.push_back( layer );
				}

				mi->mMinLayerDepth = minDepth;
				mi->mMaxLayerDepth = maxDepth;
				mBackgroundLayers.push_back( mi );

				// store model cameras
				for ( size_t i = 0; i < model->getNumCameras(); i++ )
				{
					string name = model->getCameraName( i );
					cameraNames.push_back( name );
					mCameras.push_back( model->getCamera( i ) );
				}
			}
		}
	}

	mParams.addSeparator();
	if ( mCameras.size() > 1 )
		mCameraIndex = 1;
	else
		mCameraIndex = 0;

	mParams.addParam( "Cameras", cameraNames, &mCameraIndex );
}

void BulletAssimpModelApp::startGame()
{
	mState = STATE_GAME;

	if ( mAssimpModel )
	{
		mBulletWorld.removeAssimpModel( mAssimpModel );
		mAssimpModel = NULL;
	}

	// animate the layers
	double maxDuration = 0;
	for ( auto it = mBackgroundLayers.begin(); it != mBackgroundLayers.end(); ++it )
	{
		double animDuration = (*it)->mModel->getAnimationDuration( 0 );
		(*it)->mModel->enableAnimation( true );
		maxDuration = math< double >::max( animDuration, maxDuration );
		(*it)->mTimer = 0.;
		timeline().apply( &(*it)->mTimer, animDuration, animDuration );
	}

	mIconAlpha = 0.f;
	timeline().apply( &mIconAlpha, 1.f, 2.f ).timelineEnd();

	mHandPos = Vec3f( 0, 50, 0 );
	mHandDir = Vec3f( 0, 0, 1 );
	mHandNorm = Vec3f( 0, -1, 0 );

	// add the bird after the last layer
	timeline().add( [ this ]() {
						for ( auto it = mBackgroundLayers.begin(); it != mBackgroundLayers.end(); ++it )
						{
							(*it)->mModel->enableAnimation( false );
						}
						//timeline().applyPtr( &mHandPos, Vec3f::zero(), 2.f );
						// FIXME: birth position
						mAssimpModel = mBulletWorld.spawnAssimpModel( Vec3f::zero() );
						//mBulletWorld.updateAssimpModel( mAssimpModel, mHandPos, mHandDir, mHandNorm );
					}, timeline().getCurrentTime() + maxDuration );
}

void BulletAssimpModelApp::endGame()
{
	mState = STATE_IDLE;

	if ( mAssimpModel )
	{
		mBulletWorld.removeAssimpModel( mAssimpModel );
		mAssimpModel = NULL;
	}

	mIconAlpha = 1.f;
	timeline().apply( &mIconAlpha, 0.f, 2.f );

	// animate the layers
	double maxDuration = 0;
	for ( auto it = mBackgroundLayers.begin(); it != mBackgroundLayers.end(); ++it )
	{
		double animDuration = (*it)->mModel->getAnimationDuration( 0 );
		(*it)->mModel->enableAnimation( true );
		maxDuration = math< double >::max( animDuration, maxDuration );
		(*it)->mTimer = animDuration;
		timeline().apply( &(*it)->mTimer, 0., animDuration );
	}
}

// Called when Leap frame data is ready
#if 0
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

						// FIXME: no finger getId in current LeapSdk
						/*
						if( gesture.pointable().id() == finger.getId() )
						{
							mActGestures.push_back( actFinger - 1 );
						}
						*/

						++actFinger;
					}
				}
			}
		}
	}
}
#endif

#if 0
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
#endif

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
	/*
	mParams.addPersistentParam( "Fov", &mCameraFov, 45.f, "min=20 max=180 step=.1" );
	mParams.addPersistentParam( "Eye", &mCameraEyePoint, Vec3f( 0.0f, 10.0f, -40.0f ));
	mParams.addPersistentParam( "Center of Interest", &mCameraCenterOfInterestPoint, Vec3f( 0.0f, 10.0f, 0.0f ));
	*/
	mParams.addText( "Light" );
	mParams.addPersistentParam( "Light direction", &mLightDirection, Vec3f( -0.93f, -0.27f, -0.26f ) );

	mParams.addButton( "Start (k)", std::bind( &BulletAssimpModelApp::startGame, this ) );

	mParams.addButton( "Sing (0)", [ this ]()
								{
									if( mAssimpModel )
										mAssimpModel->doAnimate( 0 );
								} );

	mParams.addButton( "Fly (1)", [ this ]()
								{
									if( mAssimpModel )
										mAssimpModel->doAnimate( 1 );
								} );

	mParams.addButton( "Wag (2)", [ this ]()
								{
									if( mAssimpModel )
										mAssimpModel->doAnimate( 2 );
								} );

	mParams.addText( "Leap" );
	mParams.addPersistentParam( "Hand pos" , &mHandPos , Vec3f( -1, -1, -1 ), "", true );
	mParams.addPersistentParam( "Hand dir" , &mHandDir , Vec3f( -1, -1, -1 ), "", true );
	mParams.addPersistentParam( "Hand norm", &mHandNorm, Vec3f( -1, -1, -1 ), "", true );
	mParams.addPersistentParam( "Smoothing", &mSmoothing, 0.95f, "min=.0 max=.99 step=.01" );
	mParams.addPersistentParam( "DrawVectors", &mDrawVectors, false );
	mParams.addSeparator();
	mParams.addPersistentParam( "Range X", &mMovementRange.x, 80, "min=0" );
	mParams.addPersistentParam( "Range Y", &mMovementRange.y, 100, "min=0" );
	mParams.addPersistentParam( "Range Z", &mMovementRange.z, 0, "min=0" );
	mParams.addPersistentParam( "Parallax scale", &mParallaxScale, 100, "step=.1" );
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
		startGame();
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

			if( mAssimpModel )
				mAssimpModel->doAnimate( pos );
		}
		break;
	default:
		mBulletWorld.keyDown( event );
	}
}

void BulletAssimpModelApp::onTap( int32_t id )
{
	//app::console() << "tap " << id << endl;
	// TODO: detect finger and play animation accordingly
	if ( mAssimpModel )
	{
		mAssimpModel->doAnimate( Rand::randInt( 3 ) );
	}
}

void BulletAssimpModelApp::update()
{
	mFps = getAverageFps();

	mBulletWorld.update();

	CameraPersp cam = mMayaCam.getCamera();
	/*
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
	*/

	mLight->setDirection( mLightDirection * Vec3f( 1.f, 1.f, -1.f ) );
	mLight->update( cam );

	updateLeap();

	if ( mAssimpModel )
		mBulletWorld.updateAssimpModel( mAssimpModel, mHandPos, mHandDir, mHandNorm );

#if 0
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

		/*
		if( ! mAssimpModel )
			mAssimpModel = mBulletWorld.spawnAssimpModel( mHandPos );
		*/

		if ( mAssimpModel )
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
#endif

/*
	if( mAssimpModelDebug )
		mBulletWorld.updateAssimpModel( mAssimpModelDebug, mPosition, mDirection.normalized(), mNormal.normalized() );
*/

#if 0
	for( Gestures::const_iterator it = mActGestures.begin(); it != mActGestures.end(); ++it )
	{
		int animPos = *it;
		if( mAssimpModelDebug )
			mAssimpModelDebug->doAnimate( animPos );
		if( mAssimpModel )
			mAssimpModel->doAnimate( animPos );
	}
#endif

	// change current camera
	static int lastCamera = -1;
	if ( lastCamera != mCameraIndex )
	{
		mMayaCam.setCurrentCam( mCameras[ mCameraIndex ] );
		lastCamera = mCameraIndex;
	}

	// update background models
	for ( auto it = mBackgroundLayers.begin(); it != mBackgroundLayers.end(); ++it )
	{
		(*it)->mModel->setTime( (*it)->mTimer );
		(*it)->mModel->update();

		// update background model layers
		float layerMove = -mParallaxScale * mHandPos.x / mMovementRange.x;
		auto layers = (*it)->mLayerNodes;
		for ( auto lit = layers.begin(); lit != layers.end(); ++lit )
		{
			const mndl::assimp::AssimpNodeRef node = (*it)->mModel->getAssimpNode( (*lit)->mName );
			float scaleX = node->getScale().x;
			if ( scaleX == 0.f )
				continue;

			float parallax = lmap< float >( (*lit)->mOrigPos.y, (*it)->mMinLayerDepth, (*it)->mMaxLayerDepth, 1.f, 0.f );
			Vec3f pos = node->getPosition(); // current position
			// set x according to parallax scrolling
			pos.x = (*lit)->mOrigPos.x + parallax * layerMove / scaleX;
			node->setPosition( pos );
		}
	}
}

void BulletAssimpModelApp::updateLeap()
{
	static double lastAppeared = 0.;
	static double lastDisappeared = 0.;
	const double handDetectedDurationThr = 2.;
	const double handRemovedDurationThr = 4.;

	double now = getElapsedSeconds();

	// query Leap
	const Leap::HandList hands = mLeapListener.getHands();
	if ( !hands.empty() )
	{
		lastAppeared = now;
		if ( ( mState == STATE_IDLE ) && ( now - lastDisappeared >= handDetectedDurationThr ) )
		{
			startGame();
		}

		// get the first hand
		const Leap::Hand hand = hands[ 0 ];
		const Leap::InteractionBox ib = mLeapListener.getInteractionBox();

		Leap::Vector npos = ib.normalizePoint( hand.palmPosition() );
		Leap::Vector ncenter = ib.normalizePoint( ib.center() );

		Vec3f handPos = mMovementRange * mndl::leap::fromLeap( npos - ncenter );
		Vec3f handDir = mndl::leap::fromLeap( hand.direction() );
		Vec3f handNorm = mndl::leap::fromLeap( hand.palmNormal() );
		mHandPos = lerp< Vec3f >( handPos, mHandPos, mSmoothing );
		mHandDir = mHandDir.slerp( mSmoothing, handDir );
		mHandNorm = mHandNorm.slerp( mSmoothing, handNorm );
		mHandDir = handDir;
		mHandNorm = handNorm;

		//const Leap::FinderList fingers = hand.fingers();
	}
	else
	{
		lastDisappeared = now;
		if ( ( mState == STATE_GAME ) && ( now - lastAppeared >= handRemovedDurationThr ) )
		{
			endGame();
		}
	}
}

void BulletAssimpModelApp::draw()
{
	gl::clear();

	gl::setViewport( getWindowBounds() );
	gl::setMatrices( mMayaCam.getCamera() );

	gl::enableDepthRead();
	gl::enableDepthWrite();

	gl::enable( GL_LIGHTING );
	mLight->enable();
	for ( auto it = mBackgroundLayers.begin(); it != mBackgroundLayers.end(); ++it )
	{
		(*it)->mModel->draw();
	}

	mBulletWorld.draw();
	mLight->disable();

	// icon layer
	gl::disable( GL_LIGHTING );
	gl::setMatricesWindow( getWindowSize() );
	gl::disableDepthRead();
	gl::disableDepthWrite();
	gl::enableAlphaBlending();
	gl::color( ColorA::gray( 1.f, mIconAlpha ) );
	Area outputArea = Area::proportionalFit( mIconLayer.getBounds(), getWindowBounds(), false, true );
	outputArea += Vec2i( 0, getWindowHeight() - outputArea.getHeight() );
	gl::draw( mIconLayer, outputArea );
	gl::disableAlphaBlending();

	mParams.draw();

	if( mDrawVectors && mAssimpModel )
	{
		Quatf rot = Quatf( Vec3f::yAxis(), M_PI / 2.0f );

		Vec3f handDir;
		Vec3f handNorm;

		if( mAssimpModel )
		{
			handDir  = rot * mHandDir;
			handNorm = rot * mHandNorm;
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
	mndl::params::PInterfaceGl::save();
}

CINDER_APP_NATIVE( BulletAssimpModelApp, RendererGl( RendererGl::AA_MSAA_4 ) )
