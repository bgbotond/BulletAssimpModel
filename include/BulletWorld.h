#ifndef __BulletWorld_H__
#define __BulletWorld_H__

#include "btBulletDynamicsCommon.h"
#include "BulletSoftBody/btSoftRigidDynamicsWorld.h"
#include "BulletSoftBody/btSoftBody.h"
#include "CinderBulletDebugDrawer.h"
#include "BulletConstraint.h"
#include "BulletParameter.h"
#include "AssimpModel.h"
#include "mndlkit/params/PParams.h"
#include "cinder/app/MouseEvent.h"
#include "cinder/Camera.h"
#include "cinder/Vector.h"

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

protected:
	btRigidBody*           createRigidBody( btScalar mass, const btTransform &startTransform, btCollisionShape *shape );
	btSoftBody*            createRope( const ci::Vec3f& from, const ci::Vec3f& to, btRigidBody* rigidBodyFrom, btRigidBody* rigidBodyTo );
	btConeTwistConstraint* createConstraint( btRigidBody& rigidBodyA, btRigidBody& rigidBodyB, const btTransform& localA, const btTransform& localB );

	void destroyRigidBody( btRigidBody* body );
	void destroyRope( btSoftBody* rope );
	void destroyConstraint( btConeTwistConstraint* constraint );

	bool checkIntersects( const ci::Ray &ray, float farClip, BulletConstraint *constraint );
	void addConstraint( const BulletConstraint &constraint, float clamping, float tau );
	void removeConstraint( const BulletConstraint &constraint );

	void setShowDebugDrawRigidBody( bool showDebugDrawRigidBody );
	bool getShowDebugDrawRigidBody() const;

protected:

	btAlignedObjectArray< class AssimpModel *> mAssimpModels;

	btDefaultCollisionConfiguration           *mCollisionConfiguration;
	btCollisionDispatcher                     *mDispatcher;
	btBroadphaseInterface                     *mBroadphase;
	btSequentialImpulseConstraintSolver       *mSolver;
	btSoftRigidDynamicsWorld                  *mSoftRigidDynamicsWorld;
	btSoftBodyWorldInfo                        mSoftBodyWorldInfo;

	btAlignedObjectArray< btCollisionShape* >  mCollisionShapes;

	std::vector< btRigidBody* >                mRigidBodies;

	CinderBulletDebugDrawer                   *mDebugDrawer;
	BulletParameter*                           mBulletParameter;
	bool                                       mShowDebugDrawRigidBody;

	double                                     mTime;

	BulletConstraint                           mBulletConstraint;
	bool                                       mDragging;

	friend class AssimpModel;
};

#endif // __BulletWorld_H__
