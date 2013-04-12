#include "cinder/app/App.h"

#include "CinderBullet.h"
#include "BulletWorld.h"

BulletWorld::BulletWorld()
: mCollisionConfiguration( NULL )
, mDispatcher( NULL )
, mBroadphase( NULL )
, mSolver( NULL )
, mSoftRigidDynamicsWorld( NULL )
, mDebugDrawer( NULL )
, mDragging( false )
{
	mBulletConstraint.init();
}

BulletWorld::~BulletWorld()
{
	donePhysics();
}

void BulletWorld::setup()
{
	mTime = ci::app::App::get()->getElapsedSeconds();
	initPhysics();
	setupParams();
	AssimpModel::setupParams();
}

void BulletWorld::update()
{
	if( mGravity != CinderBullet::convert( mSoftRigidDynamicsWorld->getGravity()))
	{
		mSoftRigidDynamicsWorld->setGravity( CinderBullet::convert( mGravity ));
		mSoftBodyWorldInfo.m_gravity = CinderBullet::convert( mGravity );
	}

	for( int i = 0; i < DEBUG_DRAW_NUM; ++i )
	{
		if( mDebugDrawActive[ i ] != mDebugDrawer->getDrawEnable( (CinderBulletDebugDrawer::DrawType)i ))
			mDebugDrawer->setDrawEnable( (CinderBulletDebugDrawer::DrawType)i, mDebugDrawActive[ i ] );
	}

	for( int i = 0; i < SOFT_DEBUG_DRAW_NUM; ++i )
	{
		if( mSoftDebugDrawActive[ i ] != mDebugDrawer->getSoftDrawEnable( (CinderBulletDebugDrawer::SoftDrawType)i ))
			mDebugDrawer->setSoftDrawEnable( (CinderBulletDebugDrawer::SoftDrawType)i, mSoftDebugDrawActive[ i ] );
	}

	mSoftRigidDynamicsWorld->setDrawFlags( mDebugDrawer->getSoftDebugMode() );

// 	// simple dynamics world doesn't handle fixed-time-stepping
// 	double time = ci::app::App::get()->getElapsedSeconds();
// 	float  ellapsedTime = ( float )( time - mTime ) * 1000000;
// 	time = mTime;
// 
// 	float minFPS = 1000000.f/60.f;
// 	if( ellapsedTime > minFPS )
// 		ellapsedTime = minFPS;
// 
// 	if( mSoftRigidDynamicsWorld )
// 	{
// 		if( mSimulateOne || mSimulateAlways )
// 		{
// 			mSoftRigidDynamicsWorld->stepSimulation( ellapsedTime / 10000.f );
// 			mSimulateOne = false;
// 		}
// 
// 		//optional but useful: debug drawing
// 		mSoftRigidDynamicsWorld->debugDrawWorld();
// 	}

	// simple dynamics world doesn't handle fixed-time-stepping
	double time = ci::app::App::get()->getElapsedSeconds();
	float  ellapsedTime = ( float )( time - mTime );
	time = mTime;

	if( mSoftRigidDynamicsWorld )
	{
		if( mSimulateOne || mSimulateAlways )
		{
//			const float timegranularityphysics = 1/60.f;
			const float timegranularityphysics = 1/10.f;
			float deltatimephy = ellapsedTime;
			if( deltatimephy > timegranularityphysics )
				deltatimephy = timegranularityphysics; // It is important to not simulate more than the granularity otherwise more than 1 round will be calculated that makes everything even slower that cause the frame time slower that cause even more round to calculate that makes it even more slower and so on until the system will stop, changing maxiteration to 1 is not a good solution, because the inner time lost would be accumulated and the system would want to get back if it can, made everything faster potentially a long period

			mSoftRigidDynamicsWorld->stepSimulation( deltatimephy, 10, timegranularityphysics );
			mSimulateOne = false;
		}

		//optional but useful: debug drawing
		mSoftRigidDynamicsWorld->debugDrawWorld();
	}
}

void BulletWorld::draw()
{
	if( mSoftRigidDynamicsWorld != NULL )
		mSoftRigidDynamicsWorld->debugDrawWorld();

	for( int i = 0; i < mAssimpModels.size(); ++i )
	{
		AssimpModel *assimpModel = mAssimpModels[ i ];

		assimpModel->draw();
	}
}

void BulletWorld::initPhysics()
{
	mCollisionConfiguration = new btDefaultCollisionConfiguration();
	mDispatcher             = new btCollisionDispatcher( mCollisionConfiguration );
	mSoftBodyWorldInfo.m_dispatcher = mDispatcher;

	btVector3 worldAabbMin( -10000, -10000, -10000 );
	btVector3 worldAabbMax(  10000,  10000,  10000 );
	mBroadphase = new btAxisSweep3( worldAabbMin, worldAabbMax );
	mSoftBodyWorldInfo.m_broadphase = mBroadphase;

	mSolver     = new btSequentialImpulseConstraintSolver;

	mSoftRigidDynamicsWorld = new btSoftRigidDynamicsWorld( mDispatcher, mBroadphase, mSolver, mCollisionConfiguration );
	mSoftRigidDynamicsWorld->setGravity( btVector3( 0, -9.81, 0 ) );
	mSoftBodyWorldInfo.m_gravity.setValue( 0, -9.81, 0 );
	mSoftBodyWorldInfo.m_sparsesdf.Initialize();

	mSoftRigidDynamicsWorld->getDispatchInfo().m_enableSPU = true;

	mDebugDrawer = new CinderBulletDebugDrawer();
	mSoftRigidDynamicsWorld->setDebugDrawer( mDebugDrawer );

	// Setup a big ground box
	{
// 		btCollisionShape *groundShape = new btBoxShape( btVector3( btScalar( 200. ), btScalar( 10. ), btScalar( 200. )));
// 
// 		mCollisionShapes.push_back( groundShape );
// 		btTransform groundTransform;
// 		groundTransform.setIdentity();
// 		groundTransform.setOrigin( btVector3( 0, -10, 0 ));
// 		createRigidBody( mSoftRigidDynamicsWorld, btScalar( 0 ), groundTransform, groundShape );
	}
}

void BulletWorld::donePhysics()
{
	for( int i = 0; i < mAssimpModels.size(); ++i )
	{
		AssimpModel *assimpModel = mAssimpModels[ i ];
		delete assimpModel;
	}

	for( int i = mSoftRigidDynamicsWorld->getNumCollisionObjects() - 1; i >= 0; --i )
	{
		btCollisionObject *obj = mSoftRigidDynamicsWorld->getCollisionObjectArray()[ i ];
		btRigidBody *body = btRigidBody::upcast( obj );
		if( body && body->getMotionState())
		{
			delete body->getMotionState();
		}

		mSoftRigidDynamicsWorld->removeCollisionObject( obj );
		delete obj;
	}

	for( int i = 0; i < mCollisionShapes.size(); ++i )
	{
		btCollisionShape *shape = mCollisionShapes[ i ];
		delete shape;
	}

	delete mSoftRigidDynamicsWorld;
	delete mSolver;
	delete mBroadphase;
	delete mDispatcher;
	delete mCollisionConfiguration;
}

btRigidBody *BulletWorld::createRigidBody( btDynamicsWorld *world, btScalar mass, const btTransform &startTransform, btCollisionShape *shape )
{
	bool isDynamic = ( mass != 0.f );

	btVector3 localInertia( 0, 0, 0 );
	if( isDynamic )
		shape->calculateLocalInertia( mass, localInertia );

	btDefaultMotionState *myMotionState = new btDefaultMotionState( startTransform );
	btRigidBody::btRigidBodyConstructionInfo rbInfo( mass, myMotionState, shape, localInertia );
	rbInfo.m_additionalDamping = true;
	btRigidBody *body = new btRigidBody( rbInfo );

	world->addRigidBody( body );//, CT_GROUND, CT_GROUND | CT_BONE );

	return body;
}

AssimpModel *BulletWorld::spawnAssimpModel( const ci::Vec3f &pos )
{
	ci::Vec3f posConv = pos / 10;

	posConv = ci::Vec3f( 0, 10, 0 );

	AssimpModel *assimpModel = new AssimpModel( mSoftRigidDynamicsWorld, &mSoftBodyWorldInfo, posConv, ci::app::App::get()->getAssetPath( "madar_0406.dae" ), ci::app::App::get()->getAssetPath( "madar_0406.xml" ) );
	mAssimpModels.push_back( assimpModel );

	return assimpModel;
}

void BulletWorld::removeAssimpModel( AssimpModel *assimpModel )
{
	mAssimpModels.remove( assimpModel );

	delete assimpModel;
}

void BulletWorld::updateAssimpModel( AssimpModel *assimpModel, const ci::Vec3f pos, const ci::Vec3f dir, const ci::Vec3f norm )
{
	ci::Vec3f posConv = pos / 10;
	assimpModel->update( posConv, dir, norm );
}

void BulletWorld::setupParams()
{
	mParams = mndl::params::PInterfaceGl( "Bullet", ci::Vec2i( 250, 350 ), ci::Vec2i( 300, 50 ) );
	mParams.addPersistentSizeAndPosition();

	mParams.addText( "World" );
	mParams.addPersistentParam( "Gravity", &mGravity, ci::Vec3f( 0.0f, -9.81f, 0.0f ) );

	mParams.addPersistentParam( "SimulateOne"   , &mSimulateOne   , false );
	mParams.addPersistentParam( "SimulateAlways", &mSimulateAlways, true  );

	mParams.addText( "DebugDraw" );
	const char *text[DEBUG_DRAW_NUM] = { "DrawWireframe", "DrawAabb", "DrawFeaturesText", "DrawContactPoints", "NoDeactivation", "NoHelpText", "DrawText", "ProfileTimings", "EnableSatComparison", "DisableBulletLCP", "EnableCCD", "DrawConstraints", "DrawConstraintLimits", "FastWireframe", "DrawNormals", "DrawTransform" };

	for( int i = 0; i < DEBUG_DRAW_NUM; ++i )
	{
		mParams.addPersistentParam( text[i], &mDebugDrawActive[i], mDebugDrawer->getDrawEnable( (CinderBulletDebugDrawer::DrawType)i ));
	}

	mParams.addText( "SoftDebugDraw" );
	const char *softText[SOFT_DEBUG_DRAW_NUM] = { "Nodes", "Links", "Faces", "Tetras", "Normals", "Contacts", "Anchors", "Notes", "Clusters", "NodeTree", "FaceTree", "ClusterTree", "Joints" };

	for( int i = 0; i < SOFT_DEBUG_DRAW_NUM; ++i )
	{
		mParams.addPersistentParam( softText[i], &mSoftDebugDrawActive[i], mDebugDrawer->getSoftDrawEnable( (CinderBulletDebugDrawer::SoftDrawType)i ));
	}
}

void BulletWorld::mouseDown( ci::app::MouseEvent event, const ci::CameraPersp &cam )
{
	ci::Vec2f pos  = ci::Vec2f( event.getPos() ) / ci::Vec2f( ci::app::App::get()->getWindowSize() );
	pos.y          = 1.0f - pos.y;
	ci::Ray ray    = cam.generateRay( pos.x, pos.y, ci::app::App::get()->getWindowAspectRatio() );
	mDragging      = checkIntersects( ray, cam.getFarClip(), &mBulletConstraint );
	if( mDragging )
	{
		addConstraint( mBulletConstraint, 0.0f, 0.01f );
	}
}

void BulletWorld::addConstraint( const BulletConstraint &constraint, float clamping, float tau )
{
	mSoftRigidDynamicsWorld->addConstraint( constraint.mConstraint );
	constraint.mConstraint->m_setting.m_impulseClamp	= clamping;
	constraint.mConstraint->m_setting.m_tau				= tau;
}

void BulletWorld::removeConstraint( const BulletConstraint &constraint )
{
	mSoftRigidDynamicsWorld->removeConstraint( constraint.mConstraint );
}

bool BulletWorld::checkIntersects( const ci::Ray &ray, float farClip, BulletConstraint *constraint )
{
	btVector3 rayFrom = CinderBullet::convert( ray.getOrigin() );
	btVector3 rayTo   = CinderBullet::convert( ray.calcPosition( farClip ) );

	btCollisionWorld::ClosestRayResultCallback rayCallback( rayFrom, rayTo );
	mSoftRigidDynamicsWorld->rayTest( rayFrom, rayTo, rayCallback );

	if( rayCallback.hasHit() )
	{
		btRigidBody* collisionBody = const_cast<btRigidBody*>( btRigidBody::upcast( rayCallback.m_collisionObject ));
		if( collisionBody )
		{
			btVector3 position = rayCallback.m_hitPointWorld;
			btVector3 pivot    = collisionBody->getCenterOfMassTransform().inverse() * position;

			constraint->mConstraint = new btPoint2PointConstraint( *collisionBody, pivot );
			constraint->mDistance   = ( position - rayFrom ).length();
			constraint->mPosition   = CinderBullet::convert( rayTo );

			return true;
		}
	}
	return false;
}

void BulletWorld::mouseDrag( ci::app::MouseEvent event, const ci::CameraPersp &cam )
{
	if( mDragging )
	{
		ci::Vec2f pos = ci::Vec2f( event.getPos() ) / ci::Vec2f( ci::app::App::get()->getWindowSize() );
		pos.y         = 1.0f - pos.y;
		ci::Ray ray   = cam.generateRay( pos.x, pos.y, ci::app::App::get()->getWindowAspectRatio() );
		mBulletConstraint.update( ray );
	}
}

void BulletWorld::mouseUp( ci::app::MouseEvent event, const ci::CameraPersp &cam )
{
	if ( mDragging )
	{
		removeConstraint( mBulletConstraint );
		mBulletConstraint.reset();
		mDragging = false;
	}
}

void BulletWorld::keyDown( ci::app::KeyEvent event )
{
	switch( event.getCode() )
	{
	case ci::app::KeyEvent::KEY_v:
		mSimulateOne = ! mSimulateOne;
		break;
	case ci::app::KeyEvent::KEY_c:
		mSimulateAlways = ! mSimulateAlways;
		break;
	}
}