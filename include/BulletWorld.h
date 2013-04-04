#ifndef __BulletWorld_H__
#define __BulletWorld_H__

#include "btBulletDynamicsCommon.h"
#include "BulletSoftBody/btSoftBody.h"
#include "CinderBulletDebugDrawer.h"
#include "BulletConstraint.h"
#include "AssimpModel.h"
#include "mndlkit/params/PParams.h"
#include "cinder/app/MouseEvent.h"
#include "cinder/Camera.h"

enum collisionTypes
{
	CT_NOTHING = 0,      // Collide with nothing
	CT_BONE    = 1 << 1, // Collide with bone
	CT_GROUND  = 2 << 1, // Collide with ground
};

class BulletWorld
{
public:
	BulletWorld();
	~BulletWorld();

	void setup();
	void update();
	void draw();

	void mouseDown( ci::app::MouseEvent event, const ci::CameraPersp &cam );
	void mouseDrag( ci::app::MouseEvent event, const ci::CameraPersp &cam );
	void mouseUp( ci::app::MouseEvent event, const ci::CameraPersp &cam );
	void keyDown( ci::app::KeyEvent event );

	void initPhysics();
	void donePhysics();

	AssimpModel *BulletWorld::spawnAssimpModel( const ci::Vec3f &pos );
	void removeAssimpModel( AssimpModel *assimpModel );
	void updateAssimpModel( AssimpModel *assimpModel, const ci::Vec3f pos, const ci::Vec3f dir, const ci::Vec3f norm );

	void animateAssimpModel( AssimpModel *assimpModel, AssimpModel::AnimateType animateType );

protected:
	btRigidBody *createRigidBody( btDynamicsWorld *world, btScalar mass, const btTransform &startTransform, btCollisionShape *shape );

	bool checkIntersects( const ci::Ray &ray, float farClip, BulletConstraint *constraint );
	void addConstraint( const BulletConstraint &constraint, float clamping, float tau );
	void removeConstraint( const BulletConstraint &constraint );

	void setupParams();

protected:

	btAlignedObjectArray< class AssimpModel *> mAssimpModels;

	btDefaultCollisionConfiguration           *mCollisionConfiguration;
	btCollisionDispatcher                     *mDispatcher;
	btBroadphaseInterface                     *mBroadphase;
	btSequentialImpulseConstraintSolver       *mSolver;
	btDynamicsWorld                           *mSoftRigidDynamicsWorld;
	btSoftBodyWorldInfo                        mSoftBodyWorldInfo;

	btAlignedObjectArray< btCollisionShape* >  mCollisionShapes;

	CinderBulletDebugDrawer                   *mDebugDrawer;

	ci::Vec3f                                  mGravity;

	double                                     mTime;
	mndl::params::PInterfaceGl                 mParams;
	static const int                           DEBUG_DRAW_NUM = 16;
	bool                                       mDebugDrawActive[ DEBUG_DRAW_NUM ];

	BulletConstraint                           mBulletConstraint;
	bool                                       mDragging;

	bool mSimulateOne;
	bool mSimulateAlways;
};

#endif // __BulletWorld_H__
