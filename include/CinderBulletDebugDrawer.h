#ifndef __CinderBulletDebugDrawer_H__
#define __CinderBulletDebugDrawer_H__

#include "btBulletCollisionCommon.h"
#include "BulletSoftBody/btSoftBodyHelpers.h"

class CinderBulletDebugDrawer : public btIDebugDraw
{
public:
	CinderBulletDebugDrawer();
	~CinderBulletDebugDrawer();

	// btIDebugDraw interface
	virtual void drawLine( const btVector3 &from, const btVector3 &to, const btVector3 &color );
	virtual void drawContactPoint( const btVector3 &PointOnB, const btVector3 &normalOnB, btScalar distance, int lifeTime, const btVector3 &color );
	virtual void reportErrorWarning( const char *warningString );
	virtual void draw3dText( const btVector3 &location, const char *textString );
	virtual void drawSphere( btScalar radius, const btTransform &transform, const btVector3 &color );
	virtual void drawCylinder( btScalar radius ,btScalar halfHeight, int upAxis, const btTransform &transform, const btVector3 &color );
	virtual void setDebugMode( int debugMode );
	virtual int  getDebugMode() const;

	void setDrawTransform( bool drawTransform );

	virtual void drawTransform( const btTransform &transform, btScalar orthoLen );

private:
	unsigned int mDebugModes;
	bool         mDrawTransform;
};


#endif // __CinderBulletDebugDrawer_H__