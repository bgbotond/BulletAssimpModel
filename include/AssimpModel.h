#ifndef __AssimpModel_H__
#define __AssimpModel_H__

#include <vector>
#include "btBulletDynamicsCommon.h"
#include "BulletSoftBody/btSoftBody.h"
#include "AssimpLoader.h"
#include "Node.h"

#include "mndlkit/params/PParams.h"

typedef std::shared_ptr< class AssimpBone             > AssimpBoneRef;
typedef std::shared_ptr< class AssimpJoint            > AssimpJointRef;
typedef std::shared_ptr< class AssimpDisableCollision > AssimpDisableCollisionRef;
typedef std::shared_ptr< class AssimpString           > AssimpStringRef;
typedef std::shared_ptr< class AssimpHang             > AssimpHangRef;
typedef std::shared_ptr< class AssimpAnimBone         > AssimpAnimBoneRef;
typedef std::shared_ptr< class AssimpAnim             > AssimpAnimRef;

class BulletWorld;

class AssimpBone
{
public:
	AssimpBone( const mndl::NodeRef& node );

	mndl::NodeRef&    getNode();

	void              setLength( const float length );
	const float       getLength() const;

	void              setMass( const float mass );
	const float       getMass() const;

	void              setShape( btCollisionShape* shape );
	btCollisionShape* getShape() const;

	void              setRigidBody( btRigidBody* rigidBody );
	btRigidBody*      getRigidBody() const;

protected:
	mndl::NodeRef     mNode;

	float             mLength;
	float             mMass;

	btCollisionShape* mShape;
	btRigidBody*      mRigidBody;
};

class AssimpJoint
{
public:
	AssimpJoint( const AssimpBoneRef& assimBoneA, const AssimpBoneRef& assimBoneB );

	const AssimpBoneRef&   getAssimpBoneA() const;
	const AssimpBoneRef&   getAssimpBoneB() const;

	void                   setSwing1( const float swing1 );
	const float            getSwing1() const;

	void                   setSwing2( const float swing2 );
	const float            getSwing2() const;

	void                   setTwist( const float twist );
	const float            getTwist() const;

	void                   setConstraint( btConeTwistConstraint* constraint );
	btConeTwistConstraint* getConstraint() const;

protected:
	AssimpBoneRef          mAssimpBoneA;
	AssimpBoneRef          mAssimpBoneB;

	float                  mSwing1;
	float                  mSwing2;
	float                  mTwist;

	btConeTwistConstraint* mConstraint;
};

class AssimpDisableCollision
{
public:
	typedef std::vector< AssimpJointRef > AssimpJoints;

public:
	AssimpDisableCollision();

	void addAssimpJoint( const AssimpJointRef& assimpJoint );
	AssimpJoints getAssimpJoints() const;

protected:
	AssimpJoints mAssimpJoints;
};

class AssimpString
{
public:
	AssimpString( const mndl::NodeRef& nodeHang, const AssimpBoneRef& assimpBonePos, const AssimpBoneRef& assimpBoneHolder );

	const mndl::NodeRef&   getNodeHang() const;
	const AssimpBoneRef&   getAssimpBonePos() const;
	const AssimpBoneRef&   getAssimpBoneHolder() const;

	void                   setRope( btSoftBody* rope );
	btSoftBody*            getRope() const;

protected:
	mndl::NodeRef     mNodeHang;
	AssimpBoneRef     mAssimpBonePos;
	AssimpBoneRef     mAssimpBoneHolder;

	btSoftBody*       mRope;
};

class AssimpHang
{
public:
	typedef std::vector< AssimpStringRef > AssimpStrings;

	AssimpHang( float stickSize, const mndl::NodeRef& nodeFront, const mndl::NodeRef& nodeBack, const mndl::NodeRef& nodeLeft, const mndl::NodeRef& nodeRigth, const mndl::NodeRef& nodeCross );

	float         getStickSize() const;

	mndl::NodeRef getNodeFront() const;
	mndl::NodeRef getNodeBack() const;
	mndl::NodeRef getNodeLeft() const;
	mndl::NodeRef getNodeRight() const;
	mndl::NodeRef getNodeCross() const;

	void             setCompoundShape( btCompoundShape* shape );
	btCompoundShape* getCompoundShape() const;
	void             setRigidBody( btRigidBody* rigidBody );
	btRigidBody*     getRigidBody() const;

	void             addAssimpString( const AssimpStringRef& assimpString );
	AssimpStrings&   getAssimpStrings();

protected:
	float            mStickSize;

	mndl::NodeRef    mNodeFront;
	mndl::NodeRef    mNodeBack;
	mndl::NodeRef    mNodeLeft;
	mndl::NodeRef    mNodeRight;
	mndl::NodeRef    mNodeCross;

	AssimpStrings    mAssimpStrings;

	btCompoundShape* mCompoundShape;
	btRigidBody*     mRigidBody;
};

class AssimpAnimBone
{
public:
	AssimpAnimBone( const AssimpBoneRef& assimpBone, const ci::Vec3f& impulse );

	AssimpBoneRef getAssimpBone() const;
	ci::Vec3f     getImpulse() const;

protected:
	AssimpBoneRef mAssimpBone;
	ci::Vec3f     mImpulse;
};

class AssimpAnim
{
public:
	typedef std::vector< AssimpAnimBoneRef > AssimpAnimBones;

	AssimpAnim( const std::string& name );

	const std::string& getName() const;

	void             addAssimpAnimBone( const AssimpAnimBoneRef& assimpAnimBone );
	AssimpAnimBones& getAssimpAnimBones();

protected:
	std::string     mName;
	AssimpAnimBones mAssimpAnimBones;
};

class AssimpModel
{
	typedef std::vector< AssimpBoneRef             > AssimpBones;
	typedef std::vector< AssimpJointRef            > AssimpJoints;
	typedef std::vector< AssimpDisableCollisionRef > AssimpDisableCollisions;
	typedef std::vector< AssimpAnimRef             > AssimpAnims;

public:
	AssimpModel( BulletWorld* bulletWorld, const ci::Vec3f &worldOffset, const boost::filesystem::path &fileModel, const boost::filesystem::path &fileData );
	~AssimpModel();

	void update( const ci::Vec3f pos, const ci::Vec3f dir, const ci::Vec3f norm );
	void draw();

	int  getNumAnimate();
	std::vector< std::string > getAnimateNames();
	void doAnimate( int pos );

protected:
	void loadData( const boost::filesystem::path& fileData );
	void loadDataBones( const ci::XmlTree& xmlNode );
	void loadDataJoints( const ci::XmlTree& xmlNode );
	void loadDataDisableCollisions( const ci::XmlTree& xmlNode );
	void loadDataDisableCollision( const ci::XmlTree& xmlNode );
	void loadDataHang( const ci::XmlTree& xmlNode );
	void loadDataString( const ci::XmlTree& xmlNode );
	void loadDataAnims( const ci::XmlTree& xmlNode );
	void loadDataAnim( const ci::XmlTree& xmlNode );

	AssimpBoneRef  getAssimpBone( const std::string& name );
	AssimpJointRef getAssimpJoint( const std::string& nameA, const std::string& nameB );

	float getLength( const mndl::NodeRef &nodeA, const mndl::NodeRef &nodeB );

	btSoftBody  *localCreateRope( const ci::Vec3f &from, const ci::Vec3f &to, btRigidBody *rigidBodyFrom, btRigidBody *rigidBodyTo );

	void buildBones();
	void buildBone( const AssimpBoneRef& assimpBone );
	void buildJoints();
	void buildJoint( const AssimpJointRef& assimpJoint );
	void buildDisableCollisions();
	void buildDisableCollision( const AssimpDisableCollisionRef& assimpDisableCollision );
	void buildHang();
	void updateBones();
	void updateBone( const AssimpBoneRef& assimpBone );
	void updateHang( const ci::Vec3f pos, const ci::Vec3f dir, const ci::Vec3f norm );

	void printNodes();
	bool isEqual( const ci::Vec3f& vec1, const ci::Vec3f& vec2 ) const;

protected:
	BulletWorld* mBulletWorld;

	mndl::assimp::AssimpLoader mAssimpLoader;

	AssimpBones             mAssimpBones;
	AssimpJoints            mAssimpJoints;
	AssimpDisableCollisions mAssimpDisableCollisions;
	AssimpHangRef           mAssimpHang;
	float                   mMinBoneLength;
	float                   mCapsuleRadius;
	AssimpAnims             mAssimpAnims;
};

#endif // __AssimpModel_H__
