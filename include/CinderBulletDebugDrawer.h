#ifndef __CinderBulletDebugDrawer_H__
#define __CinderBulletDebugDrawer_H__

#include "btBulletCollisionCommon.h"
#include "BulletSoftBody/btSoftBodyHelpers.h"

class CinderBulletDebugDrawer : public btIDebugDraw
{
public:
	enum DrawType
	{
		DT_DrawWireframe        =  0,
		DT_DrawAabb             =  1,
		DT_DrawFeaturesText     =  2,
		DT_DrawContactPoints    =  3,
		DT_NoDeactivation       =  4,
		DT_NoHelpText           =  5,
		DT_DrawText             =  6,
		DT_ProfileTimings       =  7,
		DT_EnableSatComparison  =  8,
		DT_DisableBulletLCP     =  9,
		DT_EnableCCD            = 10,
		DT_DrawConstraints      = 11,
		DT_DrawConstraintLimits = 12,
		DT_FastWireframe        = 13,
		DT_DrawNormals          = 14,
		DT_DrawTransform        = 15,
	};

	enum SoftDrawType
	{
		SDT_Nodes               =  0,
		SDT_Links               =  1,
		SDT_Faces               =  2,
		SDT_Tetras              =  3,
		SDT_Normals             =  4,
		SDT_Contacts            =  5,
		SDT_Anchors             =  6,
		SDT_Notes               =  7,
		SDT_Clusters            =  8,
		SDT_NodeTree            =  9,
		SDT_FaceTree            = 10,
		SDT_ClusterTree         = 11,
		SDT_Joints              = 12,
	};

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

	virtual void setSoftDebugMode( int softDebugMode );
	virtual int  getSoftDebugMode() const;

	virtual void drawTransform( const btTransform &transform, btScalar orthoLen );

	void setDrawEnable( DrawType drawType, bool enable );
	bool getDrawEnable( DrawType drawType              ) const;

	void setSoftDrawEnable( SoftDrawType softDrawType, bool enable );
	bool getSoftDrawEnable( SoftDrawType softDrawType              ) const;

private:
	unsigned int mDebugModes;
	unsigned int mSoftDebugModes;
	bool         mDrawTransform;
};


#endif // __CinderBulletDebugDrawer_H__