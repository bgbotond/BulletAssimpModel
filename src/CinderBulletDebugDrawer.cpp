#include "CinderBulletDebugDrawer.h"
#include "CinderBullet.h"
#include "cinder/app/App.h"
#include "cinder/gl/gl.h"

CinderBulletDebugDrawer::CinderBulletDebugDrawer()
: mDebugModes( 0 )
{
}

CinderBulletDebugDrawer::~CinderBulletDebugDrawer()
{
}

void CinderBulletDebugDrawer::drawLine( const btVector3 &from, const btVector3 &to, const btVector3 &color )
{
	ci::ColorA colorA = ci::ColorA( color.getX(), color.getY(), color.getZ() );

	ci::gl::color( colorA );
	ci::gl::drawLine( CinderBullet::convert( from ), CinderBullet::convert( to ));
}

void CinderBulletDebugDrawer::drawContactPoint( const btVector3 &pointOnB, const btVector3 &normalOnB, btScalar distance, int lifeTime, const btVector3 &color )
{
	drawLine( pointOnB, pointOnB + normalOnB * distance, color );
}

void CinderBulletDebugDrawer::reportErrorWarning( const char *warningString )
{
	ci::app::App::get()->console() << warningString << std::endl;
}

void CinderBulletDebugDrawer::draw3dText( const btVector3 &location, const char *textString )
{
	ci::gl::drawString( textString, ci::Vec2f( location.getX(), location.getY() ));
}

void CinderBulletDebugDrawer::drawSphere( btScalar radius, const btTransform &transform, const btVector3 &color )
{
	ci::ColorA colorA = ci::ColorA( color.getX(), color.getY(), color.getZ() );

	ci::gl::enableWireframe();
	ci::gl::color( colorA );
	ci::gl::pushMatrices();
	ci::gl::translate( CinderBullet::convert( transform.getOrigin()));
	ci::gl::rotate( CinderBullet::convert( transform.getRotation()));
	ci::gl::drawSphere( ci::Vec3f::zero(), radius, 20 );
	ci::gl::popMatrices();
	ci::gl::disableWireframe();
}

void CinderBulletDebugDrawer::drawCylinder( btScalar radius, btScalar halfHeight, int upAxis, const btTransform &transform, const btVector3 &color )
{
	btIDebugDraw::drawCylinder( radius, halfHeight, upAxis, transform, color );
	return;
	ci::ColorA colorA = ci::ColorA( color.getX(), color.getY(), color.getZ() );

	ci::gl::enableWireframe();
	ci::gl::color( colorA );
	ci::gl::pushMatrices();
	ci::gl::translate( CinderBullet::convert( transform.getOrigin()));
	ci::gl::rotate( CinderBullet::convert( transform.getRotation()));
	ci::gl::drawCylinder( radius, radius, halfHeight, 20, 3 );
	ci::gl::popMatrices();
	ci::gl::disableWireframe();
}

void CinderBulletDebugDrawer::setDebugMode( int debugMode )
{
	mDebugModes = (DebugDrawModes) debugMode;
}

int CinderBulletDebugDrawer::getDebugMode() const
{
	return mDebugModes;
}

void CinderBulletDebugDrawer::setDrawTransform( bool drawTransform )
{
	mDrawTransform = drawTransform;
}

void CinderBulletDebugDrawer::drawTransform( const btTransform &transform, btScalar orthoLen )
{
	if( ! mDrawTransform )
		return;

	btIDebugDraw::drawTransform( transform, orthoLen );
}

