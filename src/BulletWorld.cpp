#include "cinder/app/App.h"

#include "CinderBullet.h"
#include "BulletWorld.h"

#ifndef CONSTRAINT_DEBUG_SIZE
#define CONSTRAINT_DEBUG_SIZE 0.5f
#endif

BulletWorld::BulletWorld()
: mCollisionConfiguration( NULL )
, mDispatcher( NULL )
, mBroadphase( NULL )
, mSolver( NULL )
, mSoftRigidDynamicsWorld( NULL )
, mDebugDrawer( NULL )
, mBulletParameter( NULL )
, mDragging( false )
, mShowDebugDrawRigidBody( false )
{
	mBulletParameter = new BulletParameter();
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
	mBulletParameter->setup();
}

void BulletWorld::update()
{
	mBulletParameter->update();

	if( mBulletParameter->mGravity != CinderBullet::convert( mSoftRigidDynamicsWorld->getGravity()))
	{
		mSoftRigidDynamicsWorld->setGravity( CinderBullet::convert( mBulletParameter->mGravity ));
		mSoftBodyWorldInfo.m_gravity = CinderBullet::convert( mBulletParameter->mGravity );
	}

	setShowDebugDrawRigidBody( mBulletParameter->mShowDebugDrawRigidBody );
	mDebugDrawer->setDrawTransform( mBulletParameter->mDrawTransform );
	mDebugDrawer->setDebugMode( mBulletParameter->mGeneralDebugDrawTypes );
	mSoftRigidDynamicsWorld->setDrawFlags( mBulletParameter->mSoftDebugDrawTypes );

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
		if( mBulletParameter->mSimulateOne || mBulletParameter->mSimulateAlways )
		{
//			const float timegranularityphysics = 1/60.f;
			const float timegranularityphysics = 1/10.f;
			float deltatimephy = ellapsedTime;
			if( deltatimephy > timegranularityphysics )
				deltatimephy = timegranularityphysics; // It is important to not simulate more than the granularity otherwise more than 1 round will be calculated that makes everything even slower that cause the frame time slower that cause even more round to calculate that makes it even more slower and so on until the system will stop, changing maxiteration to 1 is not a good solution, because the inner time lost would be accumulated and the system would want to get back if it can, made everything faster potentially a long period

			mSoftRigidDynamicsWorld->stepSimulation( deltatimephy, 10, timegranularityphysics );
			mBulletParameter->mSimulateOne = false;
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
	mBulletParameter = new BulletParameter();

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
// 		createRigidBody( btScalar( 0 ), groundTransform, groundShape );
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
	delete mDebugDrawer;
	delete mBulletParameter;
}

btRigidBody *BulletWorld::createRigidBody( btScalar mass, const btTransform &startTransform, btCollisionShape *shape )
{
	bool isDynamic = ( mass != 0.f );

	btVector3 localInertia( 0, 0, 0 );
	if( isDynamic )
		shape->calculateLocalInertia( mass, localInertia );

	btDefaultMotionState* myMotionState = new btDefaultMotionState( startTransform );

	btRigidBody::btRigidBodyConstructionInfo rbInfo( mass, myMotionState, shape, localInertia );
	rbInfo.m_additionalDamping = true;
	btRigidBody* body = new btRigidBody( rbInfo );

	body->setDamping( mBulletParameter->mLinearDamping, mBulletParameter->mAngularDamping );
	body->setDeactivationTime( mBulletParameter->mDeactivationTime );
	body->setSleepingThresholds( mBulletParameter->mLinearSleepingThresholds, mBulletParameter->mAngularSleepingThresholds );

	if( mBulletParameter->mShowDebugDrawRigidBody )
		body->setCollisionFlags( body->getCollisionFlags() & ~( btCollisionObject::CF_DISABLE_VISUALIZE_OBJECT ) );
	else
		body->setCollisionFlags( body->getCollisionFlags() |    btCollisionObject::CF_DISABLE_VISUALIZE_OBJECT );

	mSoftRigidDynamicsWorld->addRigidBody( body );

	mRigidBodies.push_back( body );

	return body;
}

btSoftBody* BulletWorld::createRope( const ci::Vec3f& from, const ci::Vec3f& to, btRigidBody* rigidBodyFrom, btRigidBody* rigidBodyTo )
{
	btSoftBody* rope = btSoftBodyHelpers::CreateRope( mSoftBodyWorldInfo
		,  CinderBullet::convert( from )
		,  CinderBullet::convert( to )
		,  mBulletParameter->mRopePart
		,  2 );

	rope->setTotalMass( mBulletParameter->mRopeMass );

	rope->m_cfg.kVCF        = mBulletParameter->mKVCF;
	rope->m_cfg.kDP         = mBulletParameter->mKDP;
	rope->m_cfg.kDG         = mBulletParameter->mKDG;
	rope->m_cfg.kLF         = mBulletParameter->mKLF;
	rope->m_cfg.kPR         = mBulletParameter->mKPR;
	rope->m_cfg.kVC         = mBulletParameter->mKVC;
	rope->m_cfg.kDF         = mBulletParameter->mKDF;
	rope->m_cfg.kMT         = mBulletParameter->mKMT;
	rope->m_cfg.kCHR        = mBulletParameter->mKCHR;
	rope->m_cfg.kKHR        = mBulletParameter->mKKHR;
	rope->m_cfg.kSHR        = mBulletParameter->mKSHR;
	rope->m_cfg.kAHR        = mBulletParameter->mKAHR;
	rope->m_cfg.maxvolume   = mBulletParameter->mMaxvolume;
	rope->m_cfg.timescale   = mBulletParameter->mTimescale;
	rope->m_cfg.viterations = mBulletParameter->mViterations;
	rope->m_cfg.piterations = mBulletParameter->mPiterations;
	rope->m_cfg.diterations = mBulletParameter->mDiterations;
	rope->m_cfg.citerations = mBulletParameter->mCiterations;

	rope->appendAnchor( 0                       , rigidBodyFrom );//, CinderBullet::convert( Vec3f::zero() ) );
	rope->appendAnchor( rope->m_nodes.size() - 1, rigidBodyTo   );//, CinderBullet::convert( Vec3f::zero() ) );

	mSoftRigidDynamicsWorld->addSoftBody( rope );

	return rope;
}

btConeTwistConstraint* BulletWorld::createConstraint( btRigidBody& rigidBodyA, btRigidBody& rigidBodyB, const btTransform& localA, const btTransform& localB )
{
	btConeTwistConstraint* constraint = new btConeTwistConstraint( rigidBodyA, rigidBodyB, localA, localB );
	constraint->setDbgDrawSize( CONSTRAINT_DEBUG_SIZE );
	constraint->setDamping( mBulletParameter->mDamping );

	for( int i = 0; i < 6; ++i )
	{
		constraint->setParam( BT_CONSTRAINT_STOP_CFM, mBulletParameter->mStopCMF, i );
		constraint->setParam( BT_CONSTRAINT_STOP_ERP, mBulletParameter->mStopERP, i );
	}

	mSoftRigidDynamicsWorld->addConstraint( constraint, true );

	return constraint;
}

void BulletWorld::destroyRigidBody( btRigidBody* body )
{
	while( body->getNumConstraintRefs() > 0 )
		body->removeConstraintRef( body->getConstraintRef( 0 ) );

	mSoftRigidDynamicsWorld->removeRigidBody( body );

	if( body->getMotionState() )
		delete body->getMotionState();

	mRigidBodies.erase( std::remove( mRigidBodies.begin(), mRigidBodies.end(), body ), mRigidBodies.end() );

	delete body;
}

void BulletWorld::destroyRope( btSoftBody* rope )
{
	mSoftRigidDynamicsWorld->removeSoftBody( rope );
	delete rope;
}

void BulletWorld::destroyConstraint( btConeTwistConstraint* constraint )
{
	mSoftRigidDynamicsWorld->removeConstraint( constraint );
	delete constraint;
}

AssimpModel *BulletWorld::spawnAssimpModel( const ci::Vec3f &pos )
{
//	ci::Vec3f posConv = pos / 10;
	ci::Vec3f posConv = pos;

	posConv = ci::Vec3f( 0, 0, 0 );

	AssimpModel *assimpModel = new AssimpModel( this, posConv, ci::app::App::get()->getAssetPath( "bird.dae" ), ci::app::App::get()->getAssetPath( "bird.xml" ) );
	//AssimpModel *assimpModel = new AssimpModel( this, posConv, ci::app::App::get()->getAssetPath( "madar_0618.dae" ), ci::app::App::get()->getAssetPath( "madar_0618.xml" ) );
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
	assimpModel->update( pos, dir, norm );
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
		mBulletParameter->mSimulateOne = ! mBulletParameter->mSimulateOne;
		break;
	case ci::app::KeyEvent::KEY_c:
		mBulletParameter->mSimulateAlways = ! mBulletParameter->mSimulateAlways;
		break;
	}
}

void BulletWorld::setShowDebugDrawRigidBody( bool showDebugDrawRigidBody )
{
	if( mShowDebugDrawRigidBody == showDebugDrawRigidBody )
		return;

	mShowDebugDrawRigidBody = showDebugDrawRigidBody;

	for( std::vector< btRigidBody* >::iterator it = mRigidBodies.begin(); it != mRigidBodies.end(); ++it )
	{
		btRigidBody* rigidBody = *it;

		if( mShowDebugDrawRigidBody )
			rigidBody->setCollisionFlags( rigidBody->getCollisionFlags() & ~( btCollisionObject::CF_DISABLE_VISUALIZE_OBJECT ) );
		else
			rigidBody->setCollisionFlags( rigidBody->getCollisionFlags() |    btCollisionObject::CF_DISABLE_VISUALIZE_OBJECT );
	}
}

bool BulletWorld::getShowDebugDrawRigidBody() const
{
	return mShowDebugDrawRigidBody;
}
