#pragma warning (disable : 4996)

#include "cinder/app/App.h"
#include "cinder/CinderMath.h"
#include "cinder/Quaternion.h"
#include "cinder/Vector.h"
#include "CinderBullet.h"
#include "AssimpModel.h"
#include "BulletSoftBody/btSoftBodyHelpers.h"
#include "BulletSoftBody/btSoftRigidDynamicsWorld.h"
#include "BulletWorld.h"

#ifndef CONSTRAINT_DEBUG_SIZE
#define CONSTRAINT_DEBUG_SIZE 0.5f
#endif

using namespace ci;
using namespace mndl;
using namespace mndl::assimp;

mndl::kit::params::PInterfaceGl   AssimpModel::mParamsBird;

float                             AssimpModel::mLinearDamping             = 0.85f;
float                             AssimpModel::mAngularDamping            = 0.85f;
float                             AssimpModel::mDeactivationTime          = 0.8f;
float                             AssimpModel::mLinearSleepingThresholds  = 1.6f;
float                             AssimpModel::mAngularSleepingThresholds = 2.5f;

float                             AssimpModel::mDamping = 0.01f;
float                             AssimpModel::mStopCMF = 0.85f;
float                             AssimpModel::mStopERP = 0.65f;
// float                             AssimpModel::mLinCFM  = 0.0f;
// float                             AssimpModel::mLinERP  = 0.7f;
// float                             AssimpModel::mAngCFM  = 0.0f;


mndl::kit::params::PInterfaceGl   AssimpModel::mParamsRope;
float                             AssimpModel::mRopeSize    = 5.0f;   // rope size
int                               AssimpModel::mRopePart    = 16;     // rope part count
float                             AssimpModel::mRopeMass    = 5.0f;   // mass
float                             AssimpModel::mKVCF        = 1.0f;   // Velocities correction factor (Baumgarte)
float                             AssimpModel::mKDP         = 0.0f;   // Damping coefficient [0,1]
float                             AssimpModel::mKDG         = 0.0f;   // Drag coefficient [0,+inf]
float                             AssimpModel::mKLF         = 0.0f;   // Lift coefficient [0,+inf]
float                             AssimpModel::mKPR         = 0.0f;   // Pressure coefficient [-inf,+inf]
float                             AssimpModel::mKVC         = 0.0f;   // Volume conversation coefficient [0,+inf]
float                             AssimpModel::mKDF         = 0.2f;   // Dynamic friction coefficient [0,1]
float                             AssimpModel::mKMT         = 0.0f;   // Pose matching coefficient [0,1]
float                             AssimpModel::mKCHR        = 1.0f;   // Rigid contacts hardness [0,1]
float                             AssimpModel::mKKHR        = 0.1f;   // Kinetic contacts hardness [0,1]
float                             AssimpModel::mKSHR        = 1.0f;   // Soft contacts hardness [0,1]
float                             AssimpModel::mKAHR        = 1.0f;   // Anchors hardness [0,1]
float                             AssimpModel::mMaxvolume   = 1.0f;   // Maximum volume ratio for pose
float                             AssimpModel::mTimescale   = 1.0f;   // Time scale
int                               AssimpModel::mViterations = 0;      // Velocities solver iterations
int                               AssimpModel::mPiterations = 15;     // Positions solver iterations
int                               AssimpModel::mDiterations = 0;      // Drift solver iterations
int                               AssimpModel::mCiterations = 4;      // Cluster solver iterations

bool                              AssimpModel::mDrawSkin = true;
bool                              AssimpModel::mEnableWireframe = false;

// float                             AssimpModel::mTau          = 0.f01;
// float                             AssimpModel::mDamping      = 1.0f;
// float                             AssimpModel::mImpulseClamp = 0.0f;


AssimpModel::AssimpModel( btDynamicsWorld* ownerWorld, btSoftBodyWorldInfo* softBodyWorldInfo, const Vec3f& worldOffset, const boost::filesystem::path& fileModel, const boost::filesystem::path& fileData )
: mOwnerWorld( ownerWorld )
, mSoftBodyWorldInfo( softBodyWorldInfo )
, mMinBoneLength( 2.0f )
, mCapsuleRadius( 0.1f )
{
	mAssimpLoader = AssimpLoader( fileModel );
	mAssimpLoader.enableSkinning( true );
	mAssimpLoader.enableAnimation( true );

	loadData( fileData );

	Vec3f pos = Vec3f( 0, 15, 0 );

	AssimpNodeRef rootNode = mAssimpLoader.getRootNode();
	rootNode->setPosition( pos );

	printNodes();

	buildBones();
	buildJoints();
	buildHang();
}

void AssimpModel::loadData( const boost::filesystem::path& fileData )
{
	XmlTree doc( loadFile( fileData ) );

	if( ! doc.hasChild( "AssimpBullet" ) )
		return;

	XmlTree& node = doc.getChild( "AssimpBullet" );

	if( node.hasChild( "Bones" ))
		loadDataBones( node.getChild( "Bones" ) );

	if( node.hasChild( "Joints" ))
		loadDataJoints( node.getChild( "Joints" ) );

	if( node.hasChild( "Hang" ))
		loadDataHang( node.getChild( "Hang" ) );
}

void AssimpModel::loadDataBones( const XmlTree& xmlNode )
{
	float defMass  = xmlNode.getAttributeValue<float>( "mass",          1.0f );
	mMinBoneLength = xmlNode.getAttributeValue<float>( "minLength",     2.0f );
	mCapsuleRadius = xmlNode.getAttributeValue<float>( "capsuleRadius", 0.1f );

	for( XmlTree::ConstIter child = xmlNode.begin(); child != xmlNode.end(); ++child )
	{
		if( child->getTag() != "Bone" )
			continue;

		float mass        = defMass;
		std::string nameA = child->getAttributeValue<std::string>( "nodeA", "" );
		std::string nameB = child->getAttributeValue<std::string>( "nodeB", "" );

		if( child->hasAttribute( "mass" ) )
			mass = child->getAttributeValue<float>( "mass", 1.0f );

		mndl::NodeRef nodeA = mAssimpLoader.getAssimpNode( nameA );
		mndl::NodeRef nodeB = mAssimpLoader.getAssimpNode( nameB );

		AssimpBoneRef assimpBone = AssimpBoneRef( new AssimpBone( nodeA ) );
		assimpBone->setMass( mass );
		assimpBone->setLength( getLength( nodeA, nodeB ) );

		mAssimpBones.push_back( assimpBone ); 
	}
}

void AssimpModel::loadDataJoints( const XmlTree& xmlNode )
{
	float defSwing1 = xmlNode.getAttributeValue<float>( "swing1", 0.0f );
	float defSwing2 = xmlNode.getAttributeValue<float>( "swing2", 0.0f );
	float defTwist  = xmlNode.getAttributeValue<float>( "twist",  0.0f );

	for( XmlTree::ConstIter child = xmlNode.begin(); child != xmlNode.end(); ++child )
	{
		if( child->getTag() != "Joint" )
			continue;

		float swing1 = defSwing1;
		float swing2 = defSwing2;
		float twist  = defTwist;

		std::string nameA = child->getAttributeValue<std::string>( "nodeA", "" );
		std::string nameB = child->getAttributeValue<std::string>( "nodeB", "" );

		if( child->hasAttribute( "swing1" ) )
			swing1 = child->getAttributeValue<float>( "swing1", 0.0f );

		if( child->hasAttribute( "swing2" ) )
			swing2 = child->getAttributeValue<float>( "swing2", 0.0f );

		if( child->hasAttribute( "twist" ) )
			twist = child->getAttributeValue<float>( "twist", 0.0f );

		AssimpBoneRef assimpBoneA = getAssimpBone( nameA );
		AssimpBoneRef assimpBoneB = getAssimpBone( nameB );

		AssimpJointRef assimpJoint = AssimpJointRef( new AssimpJoint( assimpBoneA, assimpBoneB ) );
		assimpJoint->setSwing1( swing1 );
		assimpJoint->setSwing2( swing2 );
		assimpJoint->setTwist( twist );

		mAssimpJoints.push_back( assimpJoint ); 
	}
}

void AssimpModel::loadDataHang( const XmlTree& xmlNode )
{
	float stringLength = xmlNode.getAttributeValue<float>( "stringLength", 5.0f );
	float stickSize    = xmlNode.getAttributeValue<float>( "stickSize",    0.2f );

	std::string nameFront  = xmlNode.getAttributeValue<std::string>( "nodeFront",  "" );
	std::string posFront   = xmlNode.getAttributeValue<std::string>( "posFront",   "" );

	std::string nameBack   = xmlNode.getAttributeValue<std::string>( "nodeBack",   "" );
	std::string posBack    = xmlNode.getAttributeValue<std::string>( "posBack",    "" );

	std::string nameLeft   = xmlNode.getAttributeValue<std::string>( "nodeLeft",   "" );
	std::string posLeft    = xmlNode.getAttributeValue<std::string>( "posLeft",    "" );

	std::string nameRight  = xmlNode.getAttributeValue<std::string>( "nodeRight",  "" );
	std::string posRight   = xmlNode.getAttributeValue<std::string>( "posRight",   "" );

	std::string nameCenter = xmlNode.getAttributeValue<std::string>( "nodeCenter", "" );
	std::string posCenter  = xmlNode.getAttributeValue<std::string>( "posCenter",  "" );

	mAssimpHang = AssimpHangRef( new AssimpHang() );

	mAssimpHang->mStringLength     = stringLength;
	mAssimpHang->mStickSize        = stickSize;
	mAssimpHang->mAssimpBoneFront  = getAssimpBone( nameFront  );
	mAssimpHang->mAssimpBoneBack   = getAssimpBone( nameBack   );
	mAssimpHang->mAssimpBoneLeft   = getAssimpBone( nameLeft   );
	mAssimpHang->mAssimpBoneRight  = getAssimpBone( nameRight  );
	mAssimpHang->mAssimpBoneCenter = getAssimpBone( nameCenter );
	mAssimpHang->mPosTypeFront     = AssimpHang::getTypeFromString( posFront  );
	mAssimpHang->mPosTypeBack      = AssimpHang::getTypeFromString( posBack   );
	mAssimpHang->mPosTypeLeft      = AssimpHang::getTypeFromString( posLeft   );
	mAssimpHang->mPosTypeRight     = AssimpHang::getTypeFromString( posRight  );
	mAssimpHang->mPosTypeCenter    = AssimpHang::getTypeFromString( posCenter );
}

AssimpBoneRef AssimpModel::getAssimpBone( const std::string& name )
{
	for( AssimpBones::iterator it = mAssimpBones.begin(); it != mAssimpBones.end(); ++it )
	{
		AssimpBoneRef& assimpBone = *it;

		if( assimpBone->getNode() && assimpBone->getNode()->getName() == name )
			return assimpBone;
	}

	return AssimpBoneRef();
}

float AssimpModel::getLength( const mndl::NodeRef &nodeA, const mndl::NodeRef &nodeB )
{
	Vec3f pos = nodeA->getDerivedPosition();
	float lenght = mMinBoneLength;

	if( nodeB )
		lenght = ( nodeB->getDerivedPosition() - pos ).length();

	return lenght;
}

void AssimpModel::buildBones()
{
	for( AssimpBones::iterator it = mAssimpBones.begin(); it != mAssimpBones.end(); ++it )
	{
		AssimpBoneRef& assimpBone = *it;

		buildBone( assimpBone );
	}
}

void AssimpModel::buildBone( const AssimpBoneRef& assimpBone )
{
	Vec3f pos = assimpBone->getNode()->getDerivedPosition();
	Quatf rot = assimpBone->getNode()->getDerivedOrientation();
	btCollisionShape* shape = new btCapsuleShape( mCapsuleRadius, assimpBone->getLength() ); // in this way the end of the two capsules (half spheres) will be in the same position
	assimpBone->setShape( shape );

	Vec3f move = Vec3f::yAxis() * rot * ( assimpBone->getLength() / 2 );	// set the pos to the end of the capsule because in bullet the origo is in the center of the capsule

	btTransform transform;
	transform.setIdentity();
	transform.setOrigin( CinderBullet::convert( pos + move ));
	transform.setRotation( CinderBullet::convert( rot ) );
	btRigidBody* rigidBody = localCreateRigidBody( assimpBone->getMass(), transform, shape );
	assimpBone->setRigidBody( rigidBody );

	rigidBody->setDamping( mLinearDamping, mAngularDamping );
	rigidBody->setDeactivationTime( mDeactivationTime );
	rigidBody->setSleepingThresholds( mLinearSleepingThresholds, mAngularSleepingThresholds );
}

void AssimpModel::buildJoints()
{
	for( AssimpJoints::iterator it = mAssimpJoints.begin(); it != mAssimpJoints.end(); ++it )
	{
		AssimpJointRef& assimpJoint = *it;

		buildJoint( assimpJoint );
	}
}

/*
  default orientation of capsule shape is Vec3f::yAxis
  default orientation of ConeTwistConstraint is Vec3f::xAxis

                      /-\                            y
                      | |
     RigidBody B      |P|                            A
                      | |   \  /                     |
                  |   \-/    \/   Constraint         +--->  x
                  y    O
                      /-\
                      | |
     RigidBody A      |P|
                      | |
                      \-/
*/
void AssimpModel::buildJoint( const AssimpJointRef& assimpJoint )
{
	const NodeRef& nodeA = assimpJoint->getAssimpBoneA()->getNode();
	const NodeRef& nodeB = assimpJoint->getAssimpBoneB()->getNode();

	int sign = 1;
	Quatf rotB = Quatf( Vec3f::xAxis(),  Vec3f::yAxis() );
	Quatf rotA = Quatf( Vec3f::xAxis(),  Vec3f::yAxis() ) * nodeB->getOrientation();

	if( nodeA != nodeB->getParent() )
	{
		sign = -1;
		rotB = Quatf( Vec3f::xAxis(),  Vec3f::yAxis() );
		rotA = Quatf( Vec3f::xAxis(),  Vec3f::yAxis() ) * nodeB->getOrientation() * nodeA->getOrientation().inverse();
	}

	btRigidBody* rigidBodyA = assimpJoint->getAssimpBoneA()->getRigidBody();
	btRigidBody* rigidBodyB = assimpJoint->getAssimpBoneB()->getRigidBody();
	float        sizeA      = assimpJoint->getAssimpBoneA()->getLength();
	float        sizeB      = assimpJoint->getAssimpBoneB()->getLength();

	btConeTwistConstraint* coneC;
	btTransform            localA, localB;

	localA.setIdentity(); localB.setIdentity();

	localA.setRotation( CinderBullet::convert( rotA ) ); localA.setOrigin( CinderBullet::convert( Vec3f( 0,  sign * sizeA / 2, 0 ) ) );
	localB.setRotation( CinderBullet::convert( rotB ) ); localB.setOrigin( CinderBullet::convert( Vec3f( 0, -sizeB / 2, 0 ) ) );
	coneC = new btConeTwistConstraint( *rigidBodyA, *rigidBodyB, localA, localB );
	coneC->setLimit( toRadians( assimpJoint->getSwing1() ), toRadians( assimpJoint->getSwing2() ), toRadians( assimpJoint->getTwist() ) );
	coneC->setDbgDrawSize( CONSTRAINT_DEBUG_SIZE );
	coneC->setDamping( mDamping );

	for( int i = 0; i < 6; ++i )
	{
		coneC->setParam( BT_CONSTRAINT_STOP_CFM, mStopCMF, i );
		coneC->setParam( BT_CONSTRAINT_STOP_ERP, mStopERP, i );
	}

	assimpJoint->setConstraint( coneC );

	mOwnerWorld->addConstraint( coneC, true );
}

void AssimpModel::buildHang()
{
	mAssimpHang->mCompoundShape = new btCompoundShape();

	Vec3f posFront = mAssimpHang->mAssimpBoneFront->getNode()->getDerivedPosition();
	if( mAssimpHang->mPosTypeFront == AssimpHang::PT_END )
		posFront += Vec3f::yAxis() * mAssimpHang->mAssimpBoneFront->getNode()->getDerivedOrientation() * mAssimpHang->mAssimpBoneFront->getLength();

	Vec3f posBack = mAssimpHang->mAssimpBoneBack->getNode()->getDerivedPosition();
	if( mAssimpHang->mPosTypeBack == AssimpHang::PT_END )
		posBack += Vec3f::yAxis() * mAssimpHang->mAssimpBoneBack->getNode()->getDerivedOrientation() * mAssimpHang->mAssimpBoneBack->getLength();

	Vec3f posLeft = mAssimpHang->mAssimpBoneLeft->getNode()->getDerivedPosition();
	if( mAssimpHang->mPosTypeLeft == AssimpHang::PT_END )
		posLeft += Vec3f::yAxis() * mAssimpHang->mAssimpBoneLeft->getNode()->getDerivedOrientation() * mAssimpHang->mAssimpBoneLeft->getLength();

	Vec3f posRight = mAssimpHang->mAssimpBoneRight->getNode()->getDerivedPosition();
	if( mAssimpHang->mPosTypeRight == AssimpHang::PT_END )
		posRight += Vec3f::yAxis() * mAssimpHang->mAssimpBoneRight->getNode()->getDerivedOrientation() * mAssimpHang->mAssimpBoneRight->getLength();

	Vec3f posCenter = mAssimpHang->mAssimpBoneCenter->getNode()->getDerivedPosition();
	if( mAssimpHang->mPosTypeCenter == AssimpHang::PT_END )
		posCenter += Vec3f::yAxis() * mAssimpHang->mAssimpBoneCenter->getNode()->getDerivedOrientation() * mAssimpHang->mAssimpBoneCenter->getLength();

	Vec3f posCross = Vec3f( posLeft.x, posFront.y, 0 );

	mAssimpHang->mPosCross  = posCross;
	mAssimpHang->mPosFront  = posFront;
	mAssimpHang->mPosBack   = posBack;
	mAssimpHang->mPosLeft   = posLeft;
	mAssimpHang->mPosRight  = posRight;
	mAssimpHang->mPosCenter = posCenter;

	mAssimpHang->mPosCross   += Vec3f( 0, mRopeSize, 0 );
	mAssimpHang->mPosFront.y  = mAssimpHang->mPosCross.y;
	mAssimpHang->mPosBack.y   = mAssimpHang->mPosCross.y;
	mAssimpHang->mPosLeft.y   = mAssimpHang->mPosCross.y;
	mAssimpHang->mPosRight.y  = mAssimpHang->mPosCross.y;
	mAssimpHang->mPosCenter.y = mAssimpHang->mPosCross.y;

	float lengthFrontBack = ( mAssimpHang->mPosFront - mAssimpHang->mPosBack  ).length();
	float lengthLeftRight = ( mAssimpHang->mPosLeft  - mAssimpHang->mPosRight ).length();

	mAssimpHang->mShapeFrontBack = new btBoxShape( btVector3( lengthFrontBack / 2 + 2 * mAssimpHang->mStickSize, mAssimpHang->mStickSize, mAssimpHang->mStickSize ));
	mAssimpHang->mShapeLeftRight = new btBoxShape( btVector3( mAssimpHang->mStickSize, mAssimpHang->mStickSize, lengthLeftRight / 2 + 2 * mAssimpHang->mStickSize ));

	Vec3f pos;
	Quatf rot;
	btTransform transform;

	Vec3f posOffset = mAssimpHang->mPosFront - ( mAssimpHang->mPosFront - mAssimpHang->mPosBack ).normalized() * lengthFrontBack / 2;

	pos = posOffset - mAssimpHang->mPosCross;
	transform.setIdentity();
	transform.setOrigin( CinderBullet::convert( pos ));
	mAssimpHang->mCompoundShape->addChildShape( transform, mAssimpHang->mShapeFrontBack );

	pos = Vec3f::zero();
	transform.setIdentity();
	transform.setOrigin( CinderBullet::convert( pos ));
	mAssimpHang->mCompoundShape->addChildShape( transform, mAssimpHang->mShapeLeftRight );

	pos = mAssimpHang->mPosCross;
	transform.setIdentity();
	transform.setOrigin( CinderBullet::convert( pos ));
	mAssimpHang->mRigidBody = localCreateRigidBody( mAssimpHang->mStickSize * 4, transform, mAssimpHang->mCompoundShape );

	mAssimpHang->mRigidBody->setCollisionFlags( mAssimpHang->mRigidBody->getCollisionFlags() | btCollisionObject::CF_KINEMATIC_OBJECT );

	// hang rope
	Vec3f from, to;
	btRigidBody* rigidBodyFrom,* rigidBodyTo;

	from = posFront;
	to   = mAssimpHang->mPosFront;
	rigidBodyFrom = getAssimpBone( "body" )->getRigidBody();
	rigidBodyTo   = mAssimpHang->mRigidBody;
	mAssimpHang->mRopeFront = localCreateRope( from, to, rigidBodyFrom, rigidBodyTo );

	from = posBack;
	to   = mAssimpHang->mPosBack;
	rigidBodyFrom = getAssimpBone( "head" )->getRigidBody();
	rigidBodyTo   = mAssimpHang->mRigidBody;
	mAssimpHang->mRopeBack = localCreateRope( from, to, rigidBodyFrom, rigidBodyTo );

	from = posLeft;
	to   = mAssimpHang->mPosLeft;
	rigidBodyFrom = getAssimpBone( "L_foot" )->getRigidBody();
	rigidBodyTo   = mAssimpHang->mRigidBody;
	mAssimpHang->mRopeLeft = localCreateRope( from, to, rigidBodyFrom, rigidBodyTo );

	from = posRight;
	to   = mAssimpHang->mPosRight;
	rigidBodyFrom = getAssimpBone( "R_foot" )->getRigidBody();
	rigidBodyTo   = mAssimpHang->mRigidBody;
	mAssimpHang->mRopeRight = localCreateRope( from, to, rigidBodyFrom, rigidBodyTo );

	from = posCenter;
	to   = mAssimpHang->mPosCenter;
	rigidBodyFrom = getAssimpBone( "body" )->getRigidBody();
	rigidBodyTo   = mAssimpHang->mRigidBody;
	mAssimpHang->mRopeCenter = localCreateRope( from, to, rigidBodyFrom, rigidBodyTo );
}

void AssimpModel::updateBones()
{
	for( AssimpBones::iterator it = mAssimpBones.begin(); it != mAssimpBones.end(); ++it )
	{
		AssimpBoneRef& assimpBone = *it;

		updateBone( assimpBone );
	}
}

void AssimpModel::updateBone( const AssimpBoneRef& assimpBone )
{
	btMotionState* motionState = assimpBone->getRigidBody()->getMotionState();

	btTransform worldTransform;
	motionState->getWorldTransform( worldTransform );

	Vec3f pos = CinderBullet::convert( worldTransform.getOrigin() );
	Quatf rot = CinderBullet::convert( worldTransform.getRotation() );

	Vec3f move = Vec3f::yAxis() * rot * ( assimpBone->getLength() / 2 );

	NodeRef node = assimpBone->getNode();

	pos = node->convertWorldToLocalPosition( pos - move );
	rot = node->convertWorldToLocalOrientation( rot );

	node->setPosition( pos );
	node->setOrientation( rot );
}

void AssimpModel::printNodes()
{
	std::vector< std::string > names = mAssimpLoader.getNodeNames();

	for( std::vector< std::string >::iterator it = names.begin(); it != names.end(); ++it )
	{
		mndl::NodeRef node = mAssimpLoader.getAssimpNode( *it );
		mndl::NodeRef nodeParent = node->getParent();

		ci::app::App::get()->console() << "Node: " << node->getName() << " Parent: " << ( nodeParent ? nodeParent->getName() : "-" ) << " position " << node->getPosition() << " orientation " << node->getOrientation() << " scale " << node->getScale() << " derivedposition " << node->getDerivedPosition() << " derivedorientation " << node->getDerivedOrientation() << " derivedscale " << node->getDerivedScale() << std::endl;
	}
}

AssimpModel::~AssimpModel()
{
	for( AssimpJoints::iterator it = mAssimpJoints.begin(); it != mAssimpJoints.end(); ++it )
	{
		AssimpJointRef& assimpJoint = *it;

		btConeTwistConstraint* constraint = assimpJoint->getConstraint();
		assimpJoint->setConstraint( 0 );
		mOwnerWorld->removeConstraint( constraint );
		delete constraint;
	}
	mAssimpJoints.clear();

	for( AssimpBones::iterator it = mAssimpBones.begin(); it != mAssimpBones.end(); ++it )
	{
		AssimpBoneRef& assimpBone = *it;

		btRigidBody* rigidBody = assimpBone->getRigidBody();
		assimpBone->setRigidBody( 0 );

		mOwnerWorld->removeRigidBody( rigidBody );

		if( rigidBody->getMotionState() )
			delete rigidBody->getMotionState();

		delete rigidBody;

		btCollisionShape* shape = assimpBone->getShape();
		assimpBone->setShape( 0 );

		delete shape;
	}
	mAssimpBones.clear();

	{
		mOwnerWorld->removeRigidBody( mAssimpHang->mRigidBody );

		if( mAssimpHang->mRigidBody->getMotionState() )
			delete mAssimpHang->mRigidBody->getMotionState();

		delete mAssimpHang->mRigidBody;
		mAssimpHang->mRigidBody = 0;

		delete mAssimpHang->mCompoundShape;
		delete mAssimpHang->mShapeFrontBack;
		delete mAssimpHang->mShapeLeftRight;

		mAssimpHang->mCompoundShape  = 0;
		mAssimpHang->mShapeFrontBack = 0;
		mAssimpHang->mShapeLeftRight = 0;

		((btSoftRigidDynamicsWorld*)mOwnerWorld)->removeSoftBody( mAssimpHang->mRopeFront  );
		((btSoftRigidDynamicsWorld*)mOwnerWorld)->removeSoftBody( mAssimpHang->mRopeBack   );
		((btSoftRigidDynamicsWorld*)mOwnerWorld)->removeSoftBody( mAssimpHang->mRopeLeft   );
		((btSoftRigidDynamicsWorld*)mOwnerWorld)->removeSoftBody( mAssimpHang->mRopeRight  );
		((btSoftRigidDynamicsWorld*)mOwnerWorld)->removeSoftBody( mAssimpHang->mRopeCenter );

		delete mAssimpHang->mRopeFront;
		delete mAssimpHang->mRopeBack;
		delete mAssimpHang->mRopeLeft;
		delete mAssimpHang->mRopeRight;
		delete mAssimpHang->mRopeCenter;

		mAssimpHang->mRopeFront  = 0;
		mAssimpHang->mRopeBack   = 0;
		mAssimpHang->mRopeLeft   = 0;
		mAssimpHang->mRopeRight  = 0;
		mAssimpHang->mRopeCenter = 0;
	}
}

void AssimpModel::update( const Vec3f pos, const Vec3f dir, const Vec3f norm )
{
	updateBones();

	mAssimpLoader.update();

	updateHang( pos, dir, norm );
}

void AssimpModel::updateHang( const Vec3f pos, const Vec3f dir, const Vec3f norm )
{
	if( pos  == Vec3f::zero()
	 || dir  == Vec3f::zero()
	 || norm == Vec3f::zero())
		return;

	if( dir  == -Vec3f::zAxis()
	 && norm == -Vec3f::yAxis())
		return;

	Quatf rot = Quatf( Vec3f::yAxis(), -M_PI / 2.0f );
	Vec3f pos2  = rot * pos;
	Vec3f dir2  = rot * dir;
	Vec3f norm2 = rot * norm;

	Quatf normToDir( Vec3f( 0, 0, 1 ), M_PI / 2.f ); // 90 degree rotation around z
	Vec3f dir0 = norm2 * normToDir; // direction from normal
	Quatf dirQuat( dir0, dir2 ); // rotation from calculated direction to actual one
	Quatf normQuat( Vec3f( 0, -1, 0 ), norm2 ); // rotation to actual normal
	Quatf rotate = normQuat * dirQuat; // final rotation

	Vec3f posCenter = mAssimpHang->mPosCross;
	btTransform transform;
	transform.setIdentity();
	transform.setRotation( CinderBullet::convert( rotate ));
	transform.setOrigin( CinderBullet::convert( posCenter ));
	btMotionState* motionState = mAssimpHang->mRigidBody->getMotionState();
	motionState->setWorldTransform( transform );

	mAssimpHang->mRopeFront ->m_nodes[ mAssimpHang->mRopeFront ->m_nodes.size() - 1 ].m_x = CinderBullet::convert( ( rotate * ( mAssimpHang->mPosFront  - mAssimpHang->mPosCross ) ) + mAssimpHang->mPosCross );
	mAssimpHang->mRopeBack  ->m_nodes[ mAssimpHang->mRopeBack  ->m_nodes.size() - 1 ].m_x = CinderBullet::convert( ( rotate * ( mAssimpHang->mPosBack   - mAssimpHang->mPosCross ) ) + mAssimpHang->mPosCross );
	mAssimpHang->mRopeLeft  ->m_nodes[ mAssimpHang->mRopeLeft  ->m_nodes.size() - 1 ].m_x = CinderBullet::convert( ( rotate * ( mAssimpHang->mPosLeft   - mAssimpHang->mPosCross ) ) + mAssimpHang->mPosCross );
	mAssimpHang->mRopeRight ->m_nodes[ mAssimpHang->mRopeRight ->m_nodes.size() - 1 ].m_x = CinderBullet::convert( ( rotate * ( mAssimpHang->mPosRight  - mAssimpHang->mPosCross ) ) + mAssimpHang->mPosCross );
	mAssimpHang->mRopeCenter->m_nodes[ mAssimpHang->mRopeCenter->m_nodes.size() - 1 ].m_x = CinderBullet::convert( ( rotate * ( mAssimpHang->mPosCenter - mAssimpHang->mPosCross ) ) + mAssimpHang->mPosCross );
}

void AssimpModel::draw()
{
	if( mDrawSkin )
	{
		if ( mEnableWireframe )
			gl::enableWireframe();

		mAssimpLoader.draw();

		if ( mEnableWireframe )
			gl::disableWireframe();
	}
}

btRigidBody* AssimpModel::localCreateRigidBody( btScalar mass, const btTransform& startTransform, btCollisionShape* shape )
{
	bool isDynamic = ( mass != 0.f );

	btVector3 localInertia( 0, 0, 0 );
	if (isDynamic)
		shape->calculateLocalInertia( mass, localInertia );

	btDefaultMotionState* myMotionState = new btDefaultMotionState( startTransform );

	btRigidBody::btRigidBodyConstructionInfo rbInfo( mass, myMotionState, shape, localInertia );
	btRigidBody* body = new btRigidBody( rbInfo );

	mOwnerWorld->addRigidBody( body, CT_BONE, CT_GROUND );

	return body;
}

btSoftBody* AssimpModel::localCreateRope( const Vec3f& from, const Vec3f& to, btRigidBody* rigidBodyFrom, btRigidBody* rigidBodyTo )
{
	btSoftBody* rope = btSoftBodyHelpers::CreateRope( *mSoftBodyWorldInfo
	                                                ,  CinderBullet::convert( from )
	                                                ,  CinderBullet::convert( to )
	                                                ,  mRopePart
	                                                ,  2 );

	rope->setTotalMass( mRopeMass );

	rope->m_cfg.kVCF        = mKVCF;
	rope->m_cfg.kDP         = mKDP;
	rope->m_cfg.kDG         = mKDG;
	rope->m_cfg.kLF         = mKLF;
	rope->m_cfg.kPR         = mKPR;
	rope->m_cfg.kVC         = mKVC;
	rope->m_cfg.kDF         = mKDF;
	rope->m_cfg.kMT         = mKMT;
	rope->m_cfg.kCHR        = mKCHR;
	rope->m_cfg.kKHR        = mKKHR;
	rope->m_cfg.kSHR        = mKSHR;
	rope->m_cfg.kAHR        = mKAHR;
	rope->m_cfg.maxvolume   = mMaxvolume;
	rope->m_cfg.timescale   = mTimescale;
	rope->m_cfg.viterations = mViterations;
	rope->m_cfg.piterations = mPiterations;
	rope->m_cfg.diterations = mDiterations;
	rope->m_cfg.citerations = mCiterations;

	rope->appendAnchor( 0                       , rigidBodyFrom );//, CinderBullet::convert( Vec3f::zero() ) );
	rope->appendAnchor( rope->m_nodes.size() - 1, rigidBodyTo   );//, CinderBullet::convert( Vec3f::zero() ) );

	((btSoftRigidDynamicsWorld*)mOwnerWorld)->addSoftBody( rope );

	return rope;
}

void AssimpModel::setupParams()
{
	mParamsBird = mndl::kit::params::PInterfaceGl( "AssimpModel", Vec2i( 250, 350 ), Vec2i( 500, 50 ) );
	mParamsBird.addPersistentSizeAndPosition();

	mParamsBird.addText( "RigidBody" );
	mParamsBird.addPersistentParam( "Linear damping"             , &mLinearDamping            , 0.85 , "min=0.0 max=1.0 step=0.01" );
	mParamsBird.addPersistentParam( "Angular damping"            , &mAngularDamping           , 0.85 , "min=0.0 max=1.0 step=0.01" );
	mParamsBird.addPersistentParam( "Deactivation time"          , &mDeactivationTime         , 0.8  , "min=0.0         step=0.01" );
	mParamsBird.addPersistentParam( "Linear sleeping thresholds" , &mLinearSleepingThresholds , 1.6  , "min=0.0         step=0.01" );
	mParamsBird.addPersistentParam( "Angular sleeping thresholds", &mAngularSleepingThresholds, 2.5  , "min=0.0         step=0.01" );

	mParamsBird.addText( "Constraint" );
	mParamsBird.addPersistentParam( "Damping"                    , &mDamping                  , 0.60 , "min=0.0 max=1.0 step=0.01" );
	mParamsBird.addPersistentParam( "StopCMF"                    , &mStopCMF                  , 0.30 , "min=0.0 max=1.0 step=0.01" );
	mParamsBird.addPersistentParam( "StopERP"                    , &mStopERP                  , 0.80 , "min=0.0 max=1.0 step=0.01" );
// 	mParamsBird.addPersistentParam( "LinCFM"                     , &mLinCFM                   , 0.0  , "min=0.0 max=1.0 step=0.01" );
// 	mParamsBird.addPersistentParam( "LinERP"                     , &mLinERP                   , 0.7  , "min=0.0 max=1.0 step=0.01" );
// 	mParamsBird.addPersistentParam( "AngCFM"                     , &mAngCFM                   , 0.0f , "min=0.0 max=1.0 step=0.01" );

	mParamsBird.addText( "Assimp" );
	mParamsBird.addPersistentParam( "DrawSkin"                  , &mDrawSkin, true );
	mParamsBird.addPersistentParam( "EnableWireframe"           , &mEnableWireframe, false );

	mParamsRope = mndl::kit::params::PInterfaceGl( "AssimpRope", Vec2i( 250, 350 ), Vec2i( 700, 50 ) );
	mParamsRope.addPersistentSizeAndPosition();
	mParamsRope.addPersistentParam( "String size"                              , &mRopeSize    , 5.0f, "min=0.0 max=15.0 step=0.1" );
	mParamsRope.addPersistentParam( "Part"                                     , &mRopePart    , 16,   "min=4 max=50 step=1"         );
	mParamsRope.addPersistentParam( "Mass"                                     , &mRopeMass    , 5.0,  "min=0.01 max=20.0 step=0.01" );
	mParamsRope.addPersistentParam( "Velocities correction factor (Baumgarte)" , &mKVCF        , 1.0,  "min=0.0  max=20.0 step=0.1"  );
	mParamsRope.addPersistentParam( "Damping coefficient [0,1]"                , &mKDP         , 0.0,  "min=0.0  max=1.0  step=0.01" );
	mParamsRope.addPersistentParam( "Drag coefficient [0,+inf]"                , &mKDG         , 0.0,  "min=0.0           step=0.01" );
	mParamsRope.addPersistentParam( "Lift coefficient [0,+inf]"                , &mKLF         , 0.0,  "min=0.0           step=0.01" );
	mParamsRope.addPersistentParam( "Pressure coefficient [-inf,+inf]"         , &mKPR         , 0.0,  "                  step=0.01" );
	mParamsRope.addPersistentParam( "Volume conversation coefficient [0,+inf]" , &mKVC         , 0.0,  "min=0.0           step=0.01" );
	mParamsRope.addPersistentParam( "Dynamic friction coefficient [0,1]"       , &mKDF         , 0.2,  "min=0.0  max=1.0  step=0.01" );
	mParamsRope.addPersistentParam( "Pose matching coefficient [0,1]"          , &mKMT         , 0.0,  "min=0.0  max=1.0  step=0.01" );
	mParamsRope.addPersistentParam( "Rigid contacts hardness [0,1]"            , &mKCHR        , 1.0,  "min=0.0  max=1.0  step=0.01" );
	mParamsRope.addPersistentParam( "Kinetic contacts hardness [0,1]"          , &mKKHR        , 0.1,  "min=0.0  max=1.0  step=0.01" );
	mParamsRope.addPersistentParam( "Soft contacts hardness [0,1]"             , &mKSHR        , 1.0,  "min=0.0  max=1.0  step=0.01" );
	mParamsRope.addPersistentParam( "Anchors hardness [0,1]"                   , &mKAHR        , 1.0,  "min=0.0  max=1.0  step=0.01" );
	mParamsRope.addPersistentParam( "Maximum volume ratio for pose"            , &mMaxvolume   , 1.0   );
	mParamsRope.addPersistentParam( "Time scale"                               , &mTimescale   , 1.0   );
	mParamsRope.addPersistentParam( "Velocities solver iterations"             , &mViterations , 0     );
	mParamsRope.addPersistentParam( "Positions solver iterations"              , &mPiterations , 15    );
	mParamsRope.addPersistentParam( "Drift solver iterations"                  , &mDiterations , 0     );
	mParamsRope.addPersistentParam( "Cluster solver iterations"                , &mCiterations , 4     );

// 	mParams.addText( "Hang constraints" );
// 	mParams.addPersistentParam( "Tau"          , &mTau         , 0.01f,  "min=0.0 max=1.0 step=0.01" );
// 	mParams.addPersistentParam( "Damping"      , &mDamping     , 1.0f,  "min=0.0 max=1.0 step=0.01" );
// 	mParams.addPersistentParam( "Impulse clamp", &mImpulseClamp, 0.0f,  "min=0.0 max=10.0 step=0.1" );
}


/*
node: Scene - parent NULL - position [0,0,0] - orientation [-1,0,0] @ 90deg - scale [1,1,1] - derivedposition [0,0,0] - derivedorientation [-1,0,0] @ 90deg - derivedscale[1,1,1]
node: Armature - parent Scene - position [0,0,0] - orientation [0,0,0] @ 0deg - scale [1,1,1] - derivedposition [0,0,0] - derivedorientation [-1,0,0] @ 90deg - derivedscale[1,1,1]
node: body - parent Armature - position [4.35206,3.07919e-008,0.511193] - orientation [-0.619478,0.555089,0.555089] @ 116.445deg - scale [1,1,1] - derivedposition [4.35206,0.511193,3.78914e-008] - derivedorientation [-0.744752,0.667342,-1.05367e-007] @ 180deg - derivedscale[1,1,1]
node: head - parent body - position [-7.45058e-009,6.59887,-8.88178e-016] - orientation [0.0335814,-0.999385,0.0101043] @ 146.527deg - scale [1,1,1] - derivedposition [-2.20727,-0.210133,-6.15928e-008] - derivedorientation [-0.277732,0.266212,0.923036] @ 96.9984deg - derivedscale[1,1,1]
node: T_beak - parent head - position [1.49012e-008,3.73539,0] - orientation [-0.0124969,-0.998007,0.061863] @ 44.2226deg - scale [1,1,1] - derivedposition [-5.93931,-0.368282,-1.04808e-007] - derivedorientation [0.092481,-0.0845222,0.992121] @ 95.6002deg - derivedscale[1,1,1]
node: B_beak - parent head - position [1.49012e-008,3.73539,0] - orientation [-0.0331506,-0.993761,0.106489] @ 32.5935deg - scale [1,1,1] - derivedposition [-5.93931,-0.368282,-1.04808e-007] - derivedorientation [-0.00952138,0.00857103,0.999918] @ 96.0181deg - derivedscale[1,1,1]
node: R_wing - parent body - position [-7.45058e-009,6.59887,-8.88178e-016] - orientation [0.981315,-0.168849,0.0922502] @ 98.3377deg - scale [1,1,1] - derivedposition [-2.20727,-0.210133,-6.15928e-008] - derivedorientation [-0.572018,0.634305,-0.52005] @ 100.679deg - derivedscale[1,1,1]
node: L_wing - parent body - position [-7.45058e-009,6.59887,-8.88178e-016] - orientation [-0.398123,0.652607,-0.644673] @ 160.161deg - scale [1,1,1] - derivedposition [-2.20727,-0.210133,-6.15928e-008] - derivedorientation [-0.79681,-0.516678,-0.313269] @ 272.286deg - derivedscale[1,1,1]
node: L_thigh - parent body - position [-7.45058e-009,6.59887,-8.88178e-016] - orientation [0.424907,0.534927,-0.730279] @ 132.128deg - scale [1,1,1] - derivedposition [-2.20727,-0.210133,-6.15928e-008] - derivedorientation [-0.748106,-0.226505,-0.623725] @ 184.246deg - derivedscale[1,1,1]
node: L_shin - parent L_thigh - position [-1.78814e-007,5.80307,-2.38419e-007] - orientation [-0.0487576,-0.962518,-0.266799] @ 160.472deg - scale [1,1,1] - derivedposition [-0.511278,-5.40265,1.95884] - derivedorientation [-0.723952,-0.187252,0.66395] @ 229.77deg - derivedscale[1,1,1]
node: L_foot - parent L_shin - position [1.19209e-007,4.40979,4.76837e-007] - orientation [0.960339,-0.110405,0.256047] @ 65.7811deg - scale [1,1,1] - derivedposition [2.70793,-7.99624,3.49383] - derivedorientation [-0.762621,0.2896,0.578395] @ 192.031deg - derivedscale[1,1,1]
node: R_thigh - parent body - position [-7.45058e-009,6.59887,-8.88178e-016] - orientation [0.686829,0.333574,-0.64575] @ 109.899deg - scale [1,1,1] - derivedposition [-2.20727,-0.210133,-6.15928e-008] - derivedorientation [-0.803286,-0.0107562,-0.595497] @ 152.638deg - derivedscale[1,1,1]
node: R_shin - parent R_thigh - position [-5.96046e-008,5.84806,2.38419e-007] - orientation [0.0471562,0.963878,0.262137] @ 176.829deg - scale [1,1,1] - derivedposition [-0.511278,-5.40264,-2.08835] - derivedorientation [0.556113,0.413624,-0.72087] @ 156.339deg - derivedscale[1,1,1]
node: R_foot - parent R_shin - position [3.57628e-007,4.40979,0] - orientation [0.786126,0.617735,0.0202287] @ 86.6796deg - scale [1,1,1] - derivedposition [2.70793,-7.99624,-3.62334] - derivedorientation [0.852226,-0.0072194,-0.523124] @ 215.685deg - derivedscale[1,1,1]
node: tail - parent body - position [-0.0284997,0.349762,-5.5053e-008] - orientation [0.999473,-0.0193096,-0.0260835] @ 180.058deg - scale [1,1,1] - derivedposition [4.00128,0.501289,7.99921e-008] - derivedorientation [-0.0260765,-0.0302575,-0.999202] @ 81.5561deg - derivedscale[1,1,1]
node: madar - parent Scene - position [0,0,0] - orientation [0,0,0] @ 0deg - scale [1,1,1] - derivedposition [0,0,0] - derivedorientation [-1,0,0] @ 90deg - derivedscale[1,1,1]


node: Scene - parent NULL
node: Armature - parent Scene
node: body - parent Armature
node: head - parent body
node: T_beak - parent head
node: B_beak - parent head
node: R_wing - parent body
node: L_wing - parent body
node: L_thigh - parent body
node: L_shin - parent L_thigh
node: L_foot - parent L_shin
node: R_thigh - parent body
node: R_shin - parent R_thigh
node: R_foot - parent R_shin
node: tail - parent body
node: madar - parent Scene

madar_0221.dae
Scene
	madar
	Armature
		body
			head
				T_beak
				B_beak
			R_wing
			L_wing
			L_thigh
				L_shin
					L_foot
			R_thigh
				R_shin
					R_foot
			tail

------------------------------------------------------------
Node: Scene		Parent: -			position [0,15,0]								orientation [-1,0,0] @ 90deg								scale [1,1,1] derivedposition [0,15,0]							derivedorientation [-1,0,0] @ 90deg									derivedscale [1,1,1]
Node: Armature	Parent: Scene		position [0,0,0]								orientation [-1.#IND,-1.#IND,-1.#IND] @ 0deg				scale [1,1,1] derivedposition [0,15,0]							derivedorientation [-1,0,0] @ 90deg									derivedscale [1,1,1]
Node: head		Parent: Armature	position [-2.20727,3.07919e-008,-0.210134]		orientation [0.407937,-0.441408,0.799216] @ 104.004deg		scale [1,1,1] derivedposition [-2.20727,14.7899,-5.90253e-008]	derivedorientation [-0.277732,0.266212,0.923036] @ 96.9984deg		derivedscale [1,1,1]
Node: T_beak	Parent: head		position [3.72529e-008,3.73539,7.45058e-009]	orientation [-0.0251258,0.999671,-0.00507567] @ 135.87deg	scale [1,1,1] derivedposition [-5.93931,14.6317,2.21732e-007]	derivedorientation [-0.736414,0.67304,0.0686451] @ 172.82deg		derivedscale [1,1,1]
Node: B_beak	Parent: head		position [3.72529e-008,3.73539,7.45058e-009]	orientation [-0.0311165,0.999469,-0.00968673] @ 147.615deg	scale [1,1,1] derivedposition [-5.93931,14.6317,2.21732e-007]	derivedorientation [-0.743205,0.669027,-0.00707688] @ 180.73deg		derivedscale [1,1,1]
Node: R_wing	Parent: Armature	position [-2.20727,3.07919e-008,-0.210134]		orientation [0.216336,0.971572,0.0961633] @ 80.597deg		scale [1,1,1] derivedposition [-2.20727,14.7899,-5.90253e-008]	derivedorientation [-0.572018,0.634305,-0.52005] @ 100.679deg		derivedscale [1,1,1]
Node: L_wing	Parent: Armature	position [-2.20727,3.07919e-008,-0.210134]		orientation [0.408648,-0.120094,-0.904757] @ 168.561deg		scale [1,1,1] derivedposition [-2.20727,14.7899,-5.90253e-008]	derivedorientation [0.232458,-0.772251,-0.591263] @ 138.047deg		derivedscale [1,1,1]
Node: L_thigh	Parent: Armature	position [-2.20727,3.07919e-008,-0.210134]		orientation [-0.641697,0.324635,-0.694865] @ 119.678deg		scale [1,1,1] derivedposition [-2.20727,14.7899,-5.90253e-008]	derivedorientation [-0.748106,-0.226505,-0.623725] @ 184.246deg		derivedscale [1,1,1]
Node: L_shin	Parent: L_thigh		position [-2.98023e-008,5.80307,0]				orientation [-0.0487575,-0.962518,-0.266798] @ 160.472deg	scale [1,1,1] derivedposition [-0.51128,9.59735,1.95884]		derivedorientation [-0.723952,-0.187252,0.663951] @ 229.77deg		derivedscale [1,1,1]
Node: L_foot	Parent: L_shin		position [4.17233e-007,4.40979,-4.76837e-007]	orientation [0.960339,-0.110405,0.256047] @ 65.781deg		scale [1,1,1] derivedposition [2.70793,7.00376,3.49383]			derivedorientation [-0.762621,0.2896,0.578395] @ 192.031deg			derivedscale [1,1,1]
Node: R_thigh	Parent: Armature	position [-2.20727,3.07919e-008,-0.210134]		orientation [-0.553559,0.578154,-0.599425] @ 88.034deg		scale [1,1,1] derivedposition [-2.20727,14.7899,-5.90253e-008]	derivedorientation [-0.803286,-0.0107562,-0.595497] @ 152.638deg	derivedscale [1,1,1]
Node: R_shin	Parent: R_thigh		position [2.98023e-007,5.84806,2.38419e-007]	orientation [0.0471549,0.963878,0.262137] @ 176.829deg		scale [1,1,1] derivedposition [-0.51128,9.59735,-2.08835]		derivedorientation [0.556112,0.413625,-0.72087] @ 156.339deg		derivedscale [1,1,1]
Node: R_foot	Parent: R_shin		position [2.38419e-007,4.40979,0]				orientation [0.786126,0.617735,0.0202288] @ 86.6796deg		scale [1,1,1] derivedposition [2.70792,7.00376,-3.62335]		derivedorientation [0.852226,-0.00721981,-0.523125] @ 215.685deg	derivedscale [1,1,1]
Node: body		Parent: Armature	position [-2.20727,3.07919e-008,-0.210134]		orientation [0.711478,0.696473,-0.0934085] @ 160.395deg		scale [1,1,1] derivedposition [-2.20727,14.7899,-5.90253e-008]	derivedorientation [0.47656,0.533498,-0.698764] @ 103.931deg		derivedscale [1,1,1]
Node: tail		Parent: body		position [1.86265e-008,6.60115,4.47035e-008]	orientation [-0.0106398,-0.998358,-0.0562783] @ 158.623deg	scale [1,1,1] derivedposition [4.35206,15.5317,-2.56265e-006]	derivedorientation [-0.57735,-0.57735,-0.57735] @ 120deg			derivedscale [1,1,1]
Node: madar		Parent: Scene		position [0,0,0]								orientation [-1.#IND,-1.#IND,-1.#IND] @ 0deg				scale [1,1,1] derivedposition [0,15,0]							derivedorientation [-1,0,0] @ 90deg									derivedscale [1,1,1]

madar_0318.dae
Scene
	madar
	Armature
		head
			T_beak
			B_beak
		body
			tail
		R_wing
		L_wing
		L_thigh
			L_shin
				L_foot
		R_thigh
			R_shin
				R_foot
*/

AssimpBone::AssimpBone( const mndl::NodeRef& node )
: mNode( node )
, mLength( 0.0f )
, mMass( 1.0f )
, mShape( 0 )
, mRigidBody( 0 )
{
}

mndl::NodeRef& AssimpBone::getNode()
{
	return mNode;
}

void AssimpBone::setLength( const float length )
{
	mLength = length;
}

const float AssimpBone::getLength() const
{
	return mLength;
}

void AssimpBone::setMass( const float mass )
{
	mMass = mass;
}

const float AssimpBone::getMass() const
{
	return mMass;
}

void AssimpBone::setShape( btCollisionShape* shape )
{
	mShape = shape;
}

btCollisionShape* AssimpBone::getShape() const
{
	return mShape;
}

void AssimpBone::setRigidBody( btRigidBody* rigidBody )
{
	mRigidBody = rigidBody;
}

btRigidBody* AssimpBone::getRigidBody() const
{
	return mRigidBody;
}

AssimpJoint::AssimpJoint( const AssimpBoneRef& assimBoneA, const AssimpBoneRef& assimBoneB )
: mAssimpBoneA( assimBoneA )
, mAssimpBoneB( assimBoneB )
, mSwing1( 0.0f )
, mSwing2( 0.0f )
, mTwist( 0.0f )
, mConstraint( 0 )
{
}

const AssimpBoneRef& AssimpJoint::getAssimpBoneA() const
{
	return mAssimpBoneA;
}

const AssimpBoneRef& AssimpJoint::getAssimpBoneB() const
{
	return mAssimpBoneB;
}

void AssimpJoint::setSwing1( const float swing1 )
{
	mSwing1 = swing1;
}

const float AssimpJoint::getSwing1() const
{
	return mSwing1;
}

void AssimpJoint::setSwing2( const float swing2 )
{
	mSwing2 = swing2;
}

const float AssimpJoint::getSwing2() const
{
	return mSwing2;
}

void AssimpJoint::setTwist( const float twist )
{
	mTwist = twist;
}

const float AssimpJoint::getTwist() const
{
	return mTwist;
}

void AssimpJoint::setConstraint( btConeTwistConstraint* constraint )
{
	mConstraint = constraint;
}

btConeTwistConstraint* AssimpJoint::getConstraint() const
{
	return mConstraint;
}

AssimpHang::AssimpHang()
: mStringLength( 5.0f )
, mStickSize( 0.2f )
, mAssimpBoneFront()
, mPosTypeFront( PT_BEG )
, mAssimpBoneBack()
, mPosTypeBack( PT_BEG )
, mAssimpBoneLeft()
, mPosTypeLeft( PT_BEG )
, mAssimpBoneRight()
, mPosTypeRight( PT_BEG )
, mAssimpBoneCenter()
, mPosTypeCenter( PT_BEG )
, mPosFront( Vec3f::zero() )
, mPosBack( Vec3f::zero() )
, mPosLeft( Vec3f::zero() )
, mPosRight( Vec3f::zero() )
, mPosCenter( Vec3f::zero() )
, mPosCross( Vec3f::zero() )
, mCompoundShape( 0 )
, mShapeFrontBack( 0 )
, mShapeLeftRight( 0 )
, mRigidBody( 0 )
, mRopeFront( 0 )
, mRopeBack( 0 )
, mRopeLeft( 0 )
, mRopeRight( 0 )
, mRopeCenter( 0 )
{
}

AssimpHang::PosType AssimpHang::getTypeFromString( const std::string& name )
{
	/**/ if( name == "beg"    ) return PT_BEG;
	else if( name == "end"    ) return PT_END;

	return PT_BEG;
}