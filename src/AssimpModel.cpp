#pragma warning (disable : 4996)

#include "cinder/app/App.h"
#include "cinder/CinderMath.h"
#include "cinder/Quaternion.h"
#include "cinder/Vector.h"
#include "CinderBullet.h"
#include "AssimpModel.h"
#include "BulletSoftBody/btSoftBodyHelpers.h"
#include "BulletWorld.h"

using namespace ci;
using namespace mndl;
using namespace mndl::assimp;

AssimpModel::AssimpModel( BulletWorld* bulletWorld, const Vec3f& worldOffset, const boost::filesystem::path& fileModel, const boost::filesystem::path& fileData )
: mBulletWorld( bulletWorld )
, mMinBoneLength( 2.0f )
, mCapsuleRadius( 0.1f )
{
	mAssimpLoader = AssimpLoader( fileModel );
	mAssimpLoader.enableSkinning( true );
	mAssimpLoader.enableAnimation( true );

	loadData( fileData );

//	Vec3f pos = Vec3f( 0, 15, 0 );
	Vec3f pos = worldOffset;

	AssimpNodeRef rootNode = mAssimpLoader.getRootNode();
	rootNode->setPosition( pos );

//	printNodes();

	buildBones();
	buildJoints();
	buildDisableCollisions();
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

	if( node.hasChild( "DisableCollisions" ))
		loadDataDisableCollisions( node.getChild( "DisableCollisions" ) );

	if( node.hasChild( "Hang" ))
		loadDataHang( node.getChild( "Hang" ) );

	if( node.hasChild( "Anims" ))
		loadDataAnims( node.getChild( "Anims" ) );
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

void AssimpModel::loadDataDisableCollisions( const XmlTree& xmlNode )
{
	for( XmlTree::ConstIter child = xmlNode.begin(); child != xmlNode.end(); ++child )
	{
		if( child->getTag() != "DisableCollision" )
			continue;

		loadDataDisableCollision( *child );
	}
}

void AssimpModel::loadDataDisableCollision( const XmlTree& xmlNode )
{
	AssimpDisableCollisionRef assimpDisableCollision = AssimpDisableCollisionRef( new AssimpDisableCollision() );

	for( XmlTree::ConstIter child = xmlNode.begin(); child != xmlNode.end(); ++child )
	{
		if( child->getTag() != "Joint" )
			continue;

		std::string nameA = child->getAttributeValue<std::string>( "nodeA", "" );
		std::string nameB = child->getAttributeValue<std::string>( "nodeB", "" );

		AssimpJointRef assimpJoint = getAssimpJoint( nameA, nameB );

		if( assimpJoint )
			assimpDisableCollision->addAssimpJoint( assimpJoint );
	}

	if( assimpDisableCollision->getAssimpJoints().size() > 0 )
		mAssimpDisableCollisions.push_back( assimpDisableCollision ); 
}

void AssimpModel::loadDataHang( const XmlTree& xmlNode )
{
	float       stickSize = xmlNode.getAttributeValue<float>( "stickSize", 0.2f );
	std::string nameFront = xmlNode.getAttributeValue<std::string>( "nodeFront", "" );
	std::string nameBack  = xmlNode.getAttributeValue<std::string>( "nodeBack",  "" );
	std::string nameLeft  = xmlNode.getAttributeValue<std::string>( "nodeLeft",  "" );
	std::string nameRight = xmlNode.getAttributeValue<std::string>( "nodeRigth", "" );
	std::string nameCross = xmlNode.getAttributeValue<std::string>( "nodeCross", "" );

	mndl::NodeRef nodeFront = mAssimpLoader.getAssimpNode( nameFront );
	mndl::NodeRef nodeBack  = mAssimpLoader.getAssimpNode( nameBack  );
	mndl::NodeRef nodeLeft  = mAssimpLoader.getAssimpNode( nameLeft  );
	mndl::NodeRef nodeRight = mAssimpLoader.getAssimpNode( nameRight );
	mndl::NodeRef nodeCross = mAssimpLoader.getAssimpNode( nameCross );

	mAssimpHang = AssimpHangRef( new AssimpHang( stickSize, nodeFront, nodeBack, nodeLeft, nodeRight, nodeCross ) );

	for( XmlTree::ConstIter child = xmlNode.begin(); child != xmlNode.end(); ++child )
	{
		if( child->getTag() != "string" )
			continue;

		loadDataString( *child );
	}
}

void AssimpModel::loadDataString( const XmlTree& xmlNode )
{
	std::string nameNodeHang   = xmlNode.getAttributeValue<std::string>( "node",       "" );
	std::string nameBonePos    = xmlNode.getAttributeValue<std::string>( "bonePos",    "" );
	std::string nameBoneHolder = xmlNode.getAttributeValue<std::string>( "boneHolder", "" );

	mndl::NodeRef nodeHang         = mAssimpLoader.getAssimpNode( nameNodeHang );
	AssimpBoneRef assimpBonePos    = getAssimpBone( nameBonePos    );
	AssimpBoneRef assimpBoneHolder = getAssimpBone( nameBoneHolder );

	AssimpStringRef assimpString = AssimpStringRef( new AssimpString( nodeHang, assimpBonePos, assimpBoneHolder ) );

	mAssimpHang->addAssimpString( assimpString );
}

void AssimpModel::loadDataAnims( const XmlTree& xmlNode )
{
	for( XmlTree::ConstIter child = xmlNode.begin(); child != xmlNode.end(); ++child )
	{
		if( child->getTag() != "Anim" )
			continue;

		loadDataAnim( *child );
	}
}

void AssimpModel::loadDataAnim( const XmlTree& xmlNode )
{
	std::string name = xmlNode.getAttributeValue<std::string>( "name", "" );

	AssimpAnimRef assimpAnim = AssimpAnimRef( new AssimpAnim( name ) );

	for( XmlTree::ConstIter child = xmlNode.begin(); child != xmlNode.end(); ++child )
	{
		if( child->getTag() != "Bone" )
			continue;

		std::string nameBone = child->getAttributeValue<std::string>( "name", "" );
		float       rotateX  = child->getAttributeValue<float>( "rotateX", 0.0 );
		float       rotateY  = child->getAttributeValue<float>( "rotateY", 0.0 );
		float       rotateZ  = child->getAttributeValue<float>( "rotateZ", 0.0 );

		AssimpBoneRef assimpBone = getAssimpBone( nameBone );
		Vec3f         impulse    = Vec3f( rotateX, rotateY, rotateZ );

		AssimpAnimBoneRef assimpAnimBone = AssimpAnimBoneRef( new AssimpAnimBone( assimpBone, impulse ) );

		assimpAnim->addAssimpAnimBone( assimpAnimBone );
	}

	mAssimpAnims.push_back( assimpAnim );
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

AssimpJointRef AssimpModel::getAssimpJoint( const std::string& nameA, const std::string& nameB )
{
	for( AssimpJoints::iterator it = mAssimpJoints.begin(); it != mAssimpJoints.end(); ++it )
	{
		AssimpJointRef& assimpJoint = *it;

		if( assimpJoint->getAssimpBoneA()->getNode()->getName() == nameA 
		 && assimpJoint->getAssimpBoneB()->getNode()->getName() == nameB )
			return assimpJoint;
	}

	return AssimpJointRef();
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
	btRigidBody* rigidBody = mBulletWorld->createRigidBody( assimpBone->getMass(), transform, shape );
	assimpBone->setRigidBody( rigidBody );
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
	coneC = mBulletWorld->createConstraint( *rigidBodyA, *rigidBodyB, localA, localB );
	coneC->setLimit( toRadians( assimpJoint->getSwing1() ), toRadians( assimpJoint->getSwing2() ), toRadians( assimpJoint->getTwist() ) );

	assimpJoint->setConstraint( coneC );
}

void AssimpModel::buildDisableCollisions()
{
	for( AssimpDisableCollisions::iterator it = mAssimpDisableCollisions.begin(); it != mAssimpDisableCollisions.end(); ++it )
	{
		AssimpDisableCollisionRef& assimpDisableCollision = *it;

		buildDisableCollision( assimpDisableCollision );
	}
}

void AssimpModel::buildDisableCollision( const AssimpDisableCollisionRef& assimpDisableCollision )
{
	for( unsigned int pos1 = 0; pos1 < assimpDisableCollision->getAssimpJoints().size(); ++pos1 )
	{
		const AssimpJointRef assimpJoint1 = assimpDisableCollision->getAssimpJoints()[ pos1 ];

		for( unsigned int pos2 = pos1 + 1 ; pos2 < assimpDisableCollision->getAssimpJoints().size(); ++pos2 )
		{
			const AssimpJointRef assimpJoint2 = assimpDisableCollision->getAssimpJoints()[ pos2 ];

			assimpJoint1->getAssimpBoneA()->getRigidBody()->addConstraintRef( assimpJoint2->getConstraint() );
			assimpJoint1->getAssimpBoneB()->getRigidBody()->addConstraintRef( assimpJoint2->getConstraint() );

			assimpJoint2->getAssimpBoneA()->getRigidBody()->addConstraintRef( assimpJoint1->getConstraint() );
			assimpJoint2->getAssimpBoneB()->getRigidBody()->addConstraintRef( assimpJoint1->getConstraint() );
		}
	}
}

void AssimpModel::buildHang()
{
	btCompoundShape* compoundShape = new btCompoundShape();

	float lengthFrontBack = ( mAssimpHang->getNodeFront()->getDerivedPosition() - mAssimpHang->getNodeBack()->getDerivedPosition() ).length();
	float lengthLeftRight = ( mAssimpHang->getNodeLeft()->getDerivedPosition() - mAssimpHang->getNodeRight()->getDerivedPosition() ).length();

	btBoxShape* shapeFrontBack = new btBoxShape( btVector3( lengthFrontBack / 2 + 2 * mAssimpHang->getStickSize(), mAssimpHang->getStickSize(), mAssimpHang->getStickSize() ) );
	btBoxShape* shapeLeftRight = new btBoxShape( btVector3( mAssimpHang->getStickSize(), mAssimpHang->getStickSize(), lengthLeftRight / 2 + 2 * mAssimpHang->getStickSize() ) );

	Vec3f pos;
	Quatf rot;
	btTransform transform;

	Vec3f posOffset = mAssimpHang->getNodeFront()->getDerivedPosition() - ( mAssimpHang->getNodeFront()->getDerivedPosition() - mAssimpHang->getNodeBack()->getDerivedPosition() ).normalized() * lengthFrontBack / 2;

	pos = posOffset - mAssimpHang->getNodeCross()->getDerivedPosition();
	transform.setIdentity();
	transform.setOrigin( CinderBullet::convert( pos ));
	compoundShape->addChildShape( transform, shapeFrontBack );

	pos = Vec3f::zero();
	transform.setIdentity();
	transform.setOrigin( CinderBullet::convert( pos ));
	compoundShape->addChildShape( transform, shapeLeftRight );

	pos = mAssimpHang->getNodeCross()->getDerivedPosition();
	transform.setIdentity();
	transform.setOrigin( CinderBullet::convert( pos ));
	btRigidBody* rigidBody = mBulletWorld->createRigidBody( mAssimpHang->getStickSize() * 4, transform, compoundShape );

	rigidBody->setCollisionFlags( rigidBody->getCollisionFlags() | btCollisionObject::CF_KINEMATIC_OBJECT );

	mAssimpHang->setCompoundShape( compoundShape );
	mAssimpHang->setRigidBody( rigidBody );

	// hang rope
	for( AssimpHang::AssimpStrings::const_iterator it = mAssimpHang->getAssimpStrings().begin(); it != mAssimpHang->getAssimpStrings().end(); ++it )
	{
		AssimpStringRef assimpString = *it;

		Vec3f        from          = assimpString->getAssimpBonePos()->getNode()->getDerivedPosition();
		Vec3f        to            = assimpString->getNodeHang()->getDerivedPosition();
		btRigidBody* rigidBodyFrom = assimpString->getAssimpBoneHolder()->getRigidBody();
		btRigidBody* rigidBodyTo   = mAssimpHang->getRigidBody();

		btSoftBody* rope = mBulletWorld->createRope( from, to, rigidBodyFrom, rigidBodyTo );

		assimpString->setRope( rope );
	}
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

		app::App::get()->console() << "Node: " << node->getName() << " Parent: " << ( nodeParent ? nodeParent->getName() : "-" ) << " position " << node->getPosition() << " orientation " << node->getOrientation() << " scale " << node->getScale() << " derivedposition " << node->getDerivedPosition() << " derivedorientation " << node->getDerivedOrientation() << " derivedscale " << node->getDerivedScale() << std::endl;
	}
}

bool AssimpModel::isEqual( const Vec3f& vec1, const Vec3f& vec2 ) const
{
	if( math<float>::abs( vec1.x - vec2.x ) < EPSILON
	 && math<float>::abs( vec1.y - vec2.y ) < EPSILON
	 && math<float>::abs( vec1.z - vec2.z ) < EPSILON )
		return true;

	return false;
}

AssimpModel::~AssimpModel()
{
	{
		btRigidBody* rigidBody = mAssimpHang->getRigidBody();
		mAssimpHang->setRigidBody( 0 );

		mBulletWorld->destroyRigidBody( rigidBody );

		while( mAssimpHang->getCompoundShape()->getNumChildShapes() )
		{
			btCollisionShape* shape = mAssimpHang->getCompoundShape()->getChildShape( 0 );
			mAssimpHang->getCompoundShape()->removeChildShape( shape );
			delete shape;
		}

		delete mAssimpHang->getCompoundShape();
		mAssimpHang->setCompoundShape( 0 );

		for( AssimpHang::AssimpStrings::const_iterator it = mAssimpHang->getAssimpStrings().begin(); it != mAssimpHang->getAssimpStrings().end(); ++it )
		{
			AssimpStringRef assimpString = *it;
			btSoftBody* rope = assimpString->getRope();
			assimpString->setRope( 0 );

			mBulletWorld->destroyRope( rope );
		}
		mAssimpHang->getAssimpStrings().clear();
	}

	mAssimpAnims.clear();

	for( AssimpJoints::iterator it = mAssimpJoints.begin(); it != mAssimpJoints.end(); ++it )
	{
		AssimpJointRef& assimpJoint = *it;
		btConeTwistConstraint* constraint = assimpJoint->getConstraint();
		assimpJoint->setConstraint( 0 );
		mBulletWorld->destroyConstraint( constraint );
	}
	mAssimpJoints.clear();

	for( AssimpBones::iterator it = mAssimpBones.begin(); it != mAssimpBones.end(); ++it )
	{
		AssimpBoneRef& assimpBone = *it;

		btRigidBody* rigidBody = assimpBone->getRigidBody();
		assimpBone->setRigidBody( 0 );

		mBulletWorld->destroyRigidBody( rigidBody );

		btCollisionShape* shape = assimpBone->getShape();
		assimpBone->setShape( 0 );

		delete shape;
	}
	mAssimpBones.clear();
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

	if( isEqual( dir,  -Vec3f::zAxis() )
	 && isEqual( norm, -Vec3f::yAxis() ) )
		return;

	Quatf rot = Quatf( Vec3f::yAxis(), M_PI / 2.0f );
	//Vec3f pos2  = rot * pos;
	Vec3f dir2  = rot * dir;
	Vec3f norm2 = rot * norm;

	Quatf normToDir( Vec3f( 0, 0, 1 ), -M_PI / 2.f ); // 90 degree rotation around z
	Vec3f dir0 = norm2 * normToDir; // direction from normal
	Quatf dirQuat( dir0, dir2 ); // rotation from calculated direction to actual one
	Quatf normQuat( Vec3f( 0, -1, 0 ), norm2 ); // rotation to actual normal
	Quatf rotate = normQuat * dirQuat; // final rotation
	rotate.normalize();

	Vec3f posCenter = mAssimpHang->getNodeCross()->getDerivedPosition();
	Vec3f translateString = Vec3f::zero();
// 	Vec3f translateString = pos2 - mAssimpHang->getNodeCross()->getDerivedPosition();
// 	Vec3f posCenter = pos2;
	btTransform transform;
	transform.setIdentity();
	transform.setRotation( CinderBullet::convert( rotate ));
	transform.setOrigin( CinderBullet::convert( posCenter ));
	btMotionState* motionState = mAssimpHang->getRigidBody()->getMotionState();
	motionState->setWorldTransform( transform );
	mAssimpHang->getRigidBody()->setMotionState( motionState );

	for( AssimpHang::AssimpStrings::const_iterator it = mAssimpHang->getAssimpStrings().begin(); it != mAssimpHang->getAssimpStrings().end(); ++it )
	{
		AssimpStringRef assimpString = *it;

		assimpString->getRope()->m_nodes[ assimpString->getRope()->m_nodes.size() - 1 ].m_x = CinderBullet::convert( ( rotate * ( assimpString->getNodeHang()->getDerivedPosition() - mAssimpHang->getNodeCross()->getDerivedPosition() ) ) + mAssimpHang->getNodeCross()->getDerivedPosition() + translateString );
	}
}

void AssimpModel::draw()
{
	if( mBulletWorld->mBulletParameter->mDrawSkin )
	{
		if ( mBulletWorld->mBulletParameter->mEnableWireframe )
			gl::enableWireframe();

		mAssimpLoader.draw();

		if ( mBulletWorld->mBulletParameter->mEnableWireframe )
			gl::disableWireframe();
	}
}

int AssimpModel::getNumAnimate()
{
	return (int)mAssimpAnims.size();
}

std::vector< std::string > AssimpModel::getAnimateNames()
{
	std::vector< std::string > animateNames;

	for( AssimpAnims::const_iterator it = mAssimpAnims.begin(); it != mAssimpAnims.end(); ++it )
	{
		AssimpAnimRef assimpAnim = *it;
		animateNames.push_back( assimpAnim->getName() );
	}

	return animateNames;
}

void AssimpModel::doAnimate( int pos )
{
	if( pos < 0 || pos >= getNumAnimate() )
		return;

	AssimpAnimRef assimpAnim = mAssimpAnims[ pos ];

	for( AssimpAnim::AssimpAnimBones::const_iterator it = assimpAnim->getAssimpAnimBones().begin(); it != assimpAnim->getAssimpAnimBones().end(); ++it )
	{
		AssimpAnimBoneRef assimpAnimBone = *it;

		Vec3f relPos = Vec3f::yAxis() * assimpAnimBone->getAssimpBone()->getNode()->getDerivedOrientation() * ( assimpAnimBone->getAssimpBone()->getLength() / 2 );
		Vec3f impulse = relPos.cross( assimpAnimBone->getImpulse() );
		assimpAnimBone->getAssimpBone()->getRigidBody()->applyTorqueImpulse( CinderBullet::convert( impulse ) );
	}
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
	head_node
	body_node
	l_foot_node
	r_foot_node
	tail_node
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

AssimpDisableCollision::AssimpDisableCollision()
	: mAssimpJoints()
{
}

void AssimpDisableCollision::addAssimpJoint( const AssimpJointRef& assimpJoint )
{
	mAssimpJoints.push_back( assimpJoint );
}

AssimpDisableCollision::AssimpJoints AssimpDisableCollision::getAssimpJoints() const
{
	return mAssimpJoints;
}

AssimpHang::AssimpHang( float stickSize, const mndl::NodeRef& nodeFront, const mndl::NodeRef& nodeBack, const mndl::NodeRef& nodeLeft, const mndl::NodeRef& nodeRigth, const mndl::NodeRef& nodeCross )
: mStickSize( stickSize )
, mNodeFront( nodeFront )
, mNodeBack( nodeBack )
, mNodeLeft( nodeLeft )
, mNodeRight( nodeRigth )
, mNodeCross( nodeCross )
, mAssimpStrings()
, mCompoundShape()
, mRigidBody()
{
}

float AssimpHang::getStickSize() const
{
	return mStickSize;
}

mndl::NodeRef AssimpHang::getNodeFront() const
{
	return mNodeFront;
}

mndl::NodeRef AssimpHang::getNodeBack() const
{
	return mNodeBack;
}

mndl::NodeRef AssimpHang::getNodeLeft() const
{
	return mNodeLeft;
}

mndl::NodeRef AssimpHang::getNodeRight() const
{
	return mNodeRight;
}

mndl::NodeRef AssimpHang::getNodeCross() const
{
	return mNodeCross;
}

void AssimpHang::setCompoundShape( btCompoundShape* shape )
{
	mCompoundShape = shape;
}

btCompoundShape* AssimpHang::getCompoundShape() const
{
	return mCompoundShape;
}

void AssimpHang::setRigidBody( btRigidBody* rigidBody )
{
	mRigidBody = rigidBody;
}

btRigidBody* AssimpHang::getRigidBody() const
{
	return mRigidBody;
}

void AssimpHang::addAssimpString( const AssimpStringRef& assimpString )
{
	mAssimpStrings.push_back( assimpString );
}

AssimpHang::AssimpStrings& AssimpHang::getAssimpStrings()
{
	return mAssimpStrings;
}

AssimpString::AssimpString( const mndl::NodeRef& nodeHang, const AssimpBoneRef& assimpBonePos, const AssimpBoneRef& assimpBoneHolder )
: mNodeHang( nodeHang )
, mAssimpBonePos( assimpBonePos )
, mAssimpBoneHolder( assimpBoneHolder )
, mRope()
{
}

const mndl::NodeRef& AssimpString::getNodeHang() const
{
	return mNodeHang;
}

const AssimpBoneRef& AssimpString::getAssimpBonePos() const
{
	return mAssimpBonePos;
}

const AssimpBoneRef& AssimpString::getAssimpBoneHolder() const
{
	return mAssimpBoneHolder;
}

void AssimpString::setRope( btSoftBody* rope )
{
	mRope = rope;
}

btSoftBody* AssimpString::getRope() const
{
	return mRope;
}

AssimpAnimBone::AssimpAnimBone( const AssimpBoneRef& assimpBone, const Vec3f& impulse )
: mAssimpBone( assimpBone )
, mImpulse( impulse )
{
}

AssimpBoneRef AssimpAnimBone::getAssimpBone() const
{
	return mAssimpBone;
}

Vec3f AssimpAnimBone::getImpulse() const
{
	return mImpulse;
}

AssimpAnim::AssimpAnim( const std::string& name )
: mName( name )
, mAssimpAnimBones()
{
}

const std::string& AssimpAnim::getName() const
{
	return mName;
}

void AssimpAnim::addAssimpAnimBone( const AssimpAnimBoneRef& assimpAnimBone )
{
	mAssimpAnimBones.push_back( assimpAnimBone );
}

AssimpAnim::AssimpAnimBones& AssimpAnim::getAssimpAnimBones()
{
	return mAssimpAnimBones;
}
