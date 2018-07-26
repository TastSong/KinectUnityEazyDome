using UnityEngine;
//using Windows.Kinect;

using System;
using System.Collections;
using System.Collections.Generic;
using System.Runtime.InteropServices;
using System.IO;
using System.Text; 

[RequireComponent(typeof(Animator))]
public class AvatarController : MonoBehaviour
{
    // 球员的指数，模型代表的动作。默认0(第一位)
	public int playerIndex = 0;

    // 具有角色(面对玩家)动作的Bool将被镜像。默认的错误。
	public bool mirroredMovement = false;

    // Bool决定化身是否被允许做垂直运动
	public bool verticalMovement = false;

    // 《阿凡达》在场景中移动的速度。
	private int moveRate = 1;

    // Slerp光滑因子
	public float smoothFactor = 5f;
	
	
//	// Public variables that will get matched to bones. If empty, the Kinect will simply not track it.
//	public Transform HipCenter;
//	public Transform Spine;
//	public Transform ShoulderCenter;
//	public Transform Neck;
//	public Transform Head;
//
//	public Transform ClavicleLeft;
//	public Transform ShoulderLeft;
//	public Transform ElbowLeft; 
//	public Transform HandLeft;
//	public Transform FingersLeft;
//	private Transform FingerTipsLeft = null;
//	private Transform ThumbLeft = null;
//
//	public Transform ClavicleRight;
//	public Transform ShoulderRight;
//	public Transform ElbowRight;
//	public Transform HandRight;
//	public Transform FingersRight;
//	private Transform FingerTipsRight = null;
//	private Transform ThumbRight = null;
//	
//	public Transform HipLeft;
//	public Transform KneeLeft;
//	public Transform FootLeft;
//	private Transform ToesLeft = null;
//	
//	public Transform HipRight;
//	public Transform KneeRight;
//	public Transform FootRight;
//	private Transform ToesRight = null;
	
	private Transform bodyRoot;

    // 如果要在空间中旋转模型，就需要一个变量。
	private GameObject offsetNode;

    // 把所有骨头都固定住。它将初始化与初始化相同的大小。
	private Transform[] bones;

    // 当Kinect开始跟踪时骨骼的旋转。
    private Quaternion[] initialRotations;
	private Quaternion[] initialLocalRotations;

    // Kinect追踪开始时骨骼的方向。
    //private Vector3[] initialDirections;

    // 变换的初始位置和旋转
	private Vector3 initialPosition;
	private Quaternion initialRotation;

    // 校正字符位置的偏移量变量。
	private bool OffsetCalibrated = false;
	private float XOffset, YOffset, ZOffset;
	//private Quaternion originalRotation;
	
	// private instance of the KinectManager
	private KinectManager kinectManager;


    // 转换缓存可以提高性能，因为在每次调用转换时，Unity都会调用GetComponent< transform >()
	private Transform _transformCache;
	public new Transform transform
	{
		get
		{
			if (!_transformCache) _transformCache = base.transform;
			return _transformCache;
		}
	}


	public void Awake()
    {
        // 检查双开始
		if(bones != null)
			return;

        // init骨头数组
		bones = new Transform[27];

        // 骨骼的初始转动和方向。
		initialRotations = new Quaternion[bones.Length];
		initialLocalRotations = new Quaternion[bones.Length];
		//initialDirections = new Vector3[bones.Length];

        //将OffsetNode作为模型转换的父节点。只是不要每次都这么做
		offsetNode = new GameObject(name + "Ctrl") { layer = transform.gameObject.layer, tag = transform.gameObject.tag };
		offsetNode.transform.position = transform.position;
		offsetNode.transform.rotation = transform.rotation;

		transform.parent = offsetNode.transform;
		transform.localPosition = Vector3.zero;
		transform.localRotation = Quaternion.identity;
		
		bodyRoot = transform;

        // 将骨骼映射到Kinect追踪点
		MapBones();

        // 得到初始骨旋转
		GetInitialRotations();
	}

    // 更新每一帧的头像。
    public void UpdateAvatar(Int64 UserID)
    {	
		if(!transform.gameObject.activeInHierarchy) 
			return;

        // 得到KinectManager实例
		if(kinectManager == null)
		{
			kinectManager = KinectManager.Instance;
		}

        // 将化身移动到它的Kinect位置。
		MoveAvatar(UserID);

//		bool flipJoint = !mirroredMovement;
//		
//		// Update Head, Neck, Spine, and Hips
//		TransformBone(UserID, KinectInterop.JointType.SpineBase, 0, flipJoint);
//		TransformBone(UserID, KinectInterop.JointType.SpineMid, 1, flipJoint);
//		TransformBone(UserID, KinectInterop.JointType.SpineShoulder, 2, flipJoint);
//		TransformBone(UserID, KinectInterop.JointType.Neck, 3, flipJoint);
//		TransformBone(UserID, KinectInterop.JointType.Head, 4, flipJoint);
//		
//		// Beyond this, switch the arms and legs.
//		
//		// Left Arm --> Right Arm
//		TransformSpecialBone(UserID, KinectInterop.JointType.ShoulderLeft, KinectInterop.JointType.SpineShoulder, !mirroredMovement ? 25 : 26, Vector3.left, flipJoint);  // clavicle left
//		TransformBone(UserID, KinectInterop.JointType.ShoulderLeft, !mirroredMovement ? 5 : 11, flipJoint);
//		TransformBone(UserID, KinectInterop.JointType.ElbowLeft, !mirroredMovement ? 6 : 12, flipJoint);
//		TransformBone(UserID, KinectInterop.JointType.WristLeft, !mirroredMovement ? 7 : 13, flipJoint);
//		TransformBone(UserID, KinectInterop.JointType.HandLeft, !mirroredMovement ? 8 : 14, flipJoint);
//		TransformBone(UserID, KinectInterop.JointType.HandTipLeft, !mirroredMovement ? 9 : 15, flipJoint);
//		TransformBone(UserID, KinectInterop.JointType.ThumbLeft, !mirroredMovement ? 10 : 16, flipJoint);
//
//		// Right Arm --> Left Arm
//		TransformSpecialBone(UserID, KinectInterop.JointType.ShoulderRight, KinectInterop.JointType.SpineShoulder, !mirroredMovement ? 26 : 25, Vector3.right, flipJoint);  // clavicle right
//		TransformBone(UserID, KinectInterop.JointType.ShoulderRight, !mirroredMovement ? 11 : 5, flipJoint);
//		TransformBone(UserID, KinectInterop.JointType.ElbowRight, !mirroredMovement ? 12 : 6, flipJoint);
//		TransformBone(UserID, KinectInterop.JointType.WristRight, !mirroredMovement ? 13 : 7, flipJoint);
//		TransformBone(UserID, KinectInterop.JointType.HandRight, !mirroredMovement ? 14 : 8, flipJoint);
//		TransformBone(UserID, KinectInterop.JointType.HandTipRight, !mirroredMovement ? 15 : 9, flipJoint);
//		TransformBone(UserID, KinectInterop.JointType.ThumbRight, !mirroredMovement ? 16 : 10, flipJoint);
//
//		// Left Leg --> Right Leg
//		TransformBone(UserID, KinectInterop.JointType.HipLeft, !mirroredMovement ? 17 : 21, flipJoint);
//		TransformBone(UserID, KinectInterop.JointType.KneeLeft, !mirroredMovement ? 18 : 22, flipJoint);
//		TransformBone(UserID, KinectInterop.JointType.AnkleLeft, !mirroredMovement ? 19 : 23, flipJoint);
//		TransformBone(UserID, KinectInterop.JointType.FootLeft, !mirroredMovement ? 20 : 24, flipJoint);
//
//		// Right Leg --> Left Leg
//		TransformBone(UserID, KinectInterop.JointType.HipRight, !mirroredMovement ? 21 : 17, flipJoint);
//		TransformBone(UserID, KinectInterop.JointType.KneeRight, !mirroredMovement ? 22 : 18, flipJoint);
//		TransformBone(UserID, KinectInterop.JointType.AnkleRight, !mirroredMovement ? 23 : 19, flipJoint);
//		TransformBone(UserID, KinectInterop.JointType.FootRight, !mirroredMovement ? 24 : 20, flipJoint);	
		
		for (var boneIndex = 0; boneIndex < bones.Length; boneIndex++)
		{
			if (!bones[boneIndex]) 
				continue;

			if(boneIndex2JointMap.ContainsKey(boneIndex))
			{
				KinectInterop.JointType joint = !mirroredMovement ? boneIndex2JointMap[boneIndex] : boneIndex2MirrorJointMap[boneIndex];
				TransformBone(UserID, joint, boneIndex, !mirroredMovement);
			}
			else if(specIndex2JointMap.ContainsKey(boneIndex))
			{
                // 特殊的骨头(锁骨)
				List<KinectInterop.JointType> alJoints = !mirroredMovement ? specIndex2JointMap[boneIndex] : specIndex2MirrorJointMap[boneIndex];

				if(alJoints.Count >= 2)
				{
					//Debug.Log(alJoints[0].ToString());
					Vector3 baseDir = alJoints[0].ToString().EndsWith("Left") ? Vector3.left : Vector3.right;
					TransformSpecialBone(UserID, alJoints[0], alJoints[1], boneIndex, baseDir, !mirroredMovement);
				}
			}
		}
	}

    // 把骨头固定在最初的位置和转动。
	public void ResetToInitialPosition()
    {	
		if(bones == null)
			return;
		
		if(offsetNode != null)
		{
			offsetNode.transform.rotation = Quaternion.identity;
		}
		else
		{
			transform.rotation = Quaternion.identity;
		}

		//transform.position = initialPosition;
		//transform.rotation = initialRotation;

        // 对于每一块被定义的骨头，重置为初始位置。
        for (int pass = 0; pass < 2; pass++)  // 2传，因为锁骨在最后
		{
			for(int i = 0; i < bones.Length; i++)
			{
				if(bones[i] != null)
				{
					bones[i].rotation = initialRotations[i];
				}
			}
		}

		if(bodyRoot != null)
		{
			bodyRoot.localPosition = Vector3.zero;
			bodyRoot.localRotation = Quaternion.identity;
		}

        // 恢复偏移位置和旋转。
		if(offsetNode != null)
		{
			offsetNode.transform.position = initialPosition;
			offsetNode.transform.rotation = initialRotation;
		}
		else
		{
			transform.position = initialPosition;
			transform.rotation = initialRotation;
		}
    }

    // 在玩家成功校准时调用。
	public void SuccessfulCalibration(Int64 userId)
	{
        // 重置模型位置
		if(offsetNode != null)
		{
			offsetNode.transform.rotation = initialRotation;
		}
		else
		{
			transform.rotation = initialRotation;
		}

        // 珍惜的位置偏移
		OffsetCalibrated = false;
	}

    // 将kinect追踪的旋转动作应用到关节上。
	void TransformBone(Int64 userId, KinectInterop.JointType joint, int boneIndex, bool flip)
    {
		Transform boneTransform = bones[boneIndex];
		if(boneTransform == null || kinectManager == null)
			return;
		
		int iJoint = (int)joint;
		if(iJoint < 0 || !kinectManager.IsJointTracked(userId, iJoint))
			return;

        // 得到Kinect联合定位
		Quaternion jointRotation = kinectManager.GetJointOrientation(userId, iJoint, flip);
		if(jointRotation == Quaternion.identity)
			return;

        // 平稳过渡到新的旋转
		Quaternion newRotation = Kinect2AvatarRot(jointRotation, boneIndex);

		if(smoothFactor != 0f)
        	boneTransform.rotation = Quaternion.Slerp(boneTransform.rotation, newRotation, smoothFactor * Time.deltaTime);
		else
			boneTransform.rotation = newRotation;
	}

    // 将kinect追踪的旋转应用到一个特殊的关节上
	void TransformSpecialBone(Int64 userId, KinectInterop.JointType joint, KinectInterop.JointType jointParent, int boneIndex, Vector3 baseDir, bool flip)
	{
		Transform boneTransform = bones[boneIndex];
		if(boneTransform == null || kinectManager == null)
			return;
		
		if(!kinectManager.IsJointTracked(userId, (int)joint) || 
		   !kinectManager.IsJointTracked(userId, (int)jointParent))
		{
			return;
		}
		
		Vector3 jointDir = kinectManager.GetJointDirection(userId, (int)joint, false, true);
		Quaternion jointRotation = Quaternion.FromToRotation(baseDir, jointDir);
		
		if(!flip)
		{
			Vector3 mirroredAngles = jointRotation.eulerAngles;
			mirroredAngles.y = -mirroredAngles.y;
			mirroredAngles.z = -mirroredAngles.z;

			jointRotation = Quaternion.Euler(mirroredAngles);
		}
		
		if(jointRotation != Quaternion.identity)
		{
            // 平稳过渡到新的旋转
			Quaternion newRotation = Kinect2AvatarRot(jointRotation, boneIndex);
			
			if(smoothFactor != 0f)
				boneTransform.rotation = Quaternion.Slerp(boneTransform.rotation, newRotation, smoothFactor * Time.deltaTime);
			else
				boneTransform.rotation = newRotation;
		}
		
	}

    // 在3D空间移动化身——拉出用户被跟踪的位置并应用到root。
	void MoveAvatar(Int64 UserID)
	{
		if(!kinectManager || !kinectManager.IsJointTracked(UserID, (int)KinectInterop.JointType.SpineBase))
			return;

        // 获得身体的位置并储存它。
		Vector3 trans = kinectManager.GetUserPosition(UserID);

        // 如果这是我们第一次移动头像，设置偏移量。否则忽略它。
		if (!OffsetCalibrated)
		{
			OffsetCalibrated = true;
			
			XOffset = !mirroredMovement ? trans.x * moveRate : -trans.x * moveRate;
			YOffset = trans.y * moveRate;
			ZOffset = -trans.z * moveRate;
		}

        // 顺利地过渡到新的岗位
		Vector3 targetPos = Kinect2AvatarPos(trans, verticalMovement);

		if(bodyRoot != null)
		{
			bodyRoot.localPosition = smoothFactor != 0f ? 
				Vector3.Lerp(bodyRoot.localPosition, targetPos, smoothFactor * Time.deltaTime) : targetPos;
		}
		else
		{
			transform.localPosition = smoothFactor != 0f ? 
				Vector3.Lerp(transform.localPosition, targetPos, smoothFactor * Time.deltaTime) : targetPos;
		}
	}

    // 如果要映射的骨头已经声明，将该骨头映射到模型。
	void MapBones()
	{
//		bones[0] = HipCenter;
//		bones[1] = Spine;
//		bones[2] = ShoulderCenter;
//		bones[3] = Neck;
//		bones[4] = Head;
//	
//		bones[5] = ShoulderLeft;
//		bones[6] = ElbowLeft;
//		bones[7] = HandLeft;
//		bones[8] = FingersLeft;
//		bones[9] = FingerTipsLeft;
//		bones[10] = ThumbLeft;
//	
//		bones[11] = ShoulderRight;
//		bones[12] = ElbowRight;
//		bones[13] = HandRight;
//		bones[14] = FingersRight;
//		bones[15] = FingerTipsRight;
//		bones[16] = ThumbRight;
//	
//		bones[17] = HipLeft;
//		bones[18] = KneeLeft;
//		bones[19] = FootLeft;
//		bones[20] = ToesLeft;
//	
//		bones[21] = HipRight;
//		bones[22] = KneeRight;
//		bones[23] = FootRight;
//		bones[24] = ToesRight;
//
//		// special bones
//		bones[25] = ClavicleLeft;
//		bones[26] = ClavicleRight;

		var animatorComponent = GetComponent<Animator>();
		
		for (int boneIndex = 0; boneIndex < bones.Length; boneIndex++)
		{
			if (!boneIndex2MecanimMap.ContainsKey(boneIndex)) 
				continue;
			
			//_bones[kinectInt] = ClosestMecanimBoneByKinectId(kinectId, animatorComponent);
			bones[boneIndex] = animatorComponent.GetBoneTransform(boneIndex2MecanimMap[boneIndex]);
		}
	}

    // 捕捉骨骼最初的旋转
	void GetInitialRotations()
	{
        // 保存最初的旋转
		if(offsetNode != null)
		{
			initialPosition = offsetNode.transform.position;
			initialRotation = offsetNode.transform.rotation;

			offsetNode.transform.rotation = Quaternion.identity;
		}
		else
		{
			initialPosition = transform.position;
			initialRotation = transform.rotation;

			transform.rotation = Quaternion.identity;
		}

		for (int i = 0; i < bones.Length; i++)
		{
			if (bones[i] != null)
			{
				initialRotations[i] = bones[i].rotation; // * Quaternion.Inverse(initialRotation);
				initialLocalRotations[i] = bones[i].localRotation;
			}
		}

        // 恢复最初的旋转
		if(offsetNode != null)
		{
			offsetNode.transform.rotation = initialRotation;
		}
		else
		{
			transform.rotation = initialRotation;
		}
	}

    //将kinect关节旋转转换为avatar关节旋转，取决于关节初始旋转和偏移旋转
	Quaternion Kinect2AvatarRot(Quaternion jointRotation, int boneIndex)
	{
		Quaternion newRotation = jointRotation * initialRotations[boneIndex];

		if (offsetNode != null)
		{
			Vector3 totalRotation = newRotation.eulerAngles + offsetNode.transform.rotation.eulerAngles;
			newRotation = Quaternion.Euler(totalRotation);
		}
		
		return newRotation;
	}

    // 将Kinect位置转换为avatar骨架位置，取决于初始位置、镜像和移动速度
	Vector3 Kinect2AvatarPos(Vector3 jointPosition, bool bMoveVertically)
	{
		float xPos;

		if(!mirroredMovement)
			xPos = jointPosition.x * moveRate - XOffset;
		else
			xPos = -jointPosition.x * moveRate - XOffset;
		
		float yPos = jointPosition.y * moveRate - YOffset;
		float zPos = -jointPosition.z * moveRate - ZOffset;
		
		Vector3 newPosition = new Vector3(xPos, bMoveVertically ? yPos : 0f, zPos);

		return newPosition;
	}

    //字典可以加速骨骼的处理
    //这是关于运动关节到骨骼映射的好主意的作者
    //连同它的初始实现，包括下面的dictionary is
    //米哈伊尔·Korchun(korchoon@gmail.com)。非常感谢这个家伙!
	private readonly Dictionary<int, HumanBodyBones> boneIndex2MecanimMap = new Dictionary<int, HumanBodyBones>
	{
		{0, HumanBodyBones.Hips},
		{1, HumanBodyBones.Spine},
//        {2, HumanBodyBones.Chest},
		{3, HumanBodyBones.Neck},
		{4, HumanBodyBones.Head},
		
		{5, HumanBodyBones.LeftUpperArm},
		{6, HumanBodyBones.LeftLowerArm},
		{7, HumanBodyBones.LeftHand},
		{8, HumanBodyBones.LeftIndexProximal},
		
		{9, HumanBodyBones.LeftIndexIntermediate},
		{10, HumanBodyBones.LeftThumbProximal},
		
		{11, HumanBodyBones.RightUpperArm},
		{12, HumanBodyBones.RightLowerArm},
		{13, HumanBodyBones.RightHand},
		{14, HumanBodyBones.RightIndexProximal},
		
		{15, HumanBodyBones.RightIndexIntermediate},
		{16, HumanBodyBones.RightThumbProximal},
		
		{17, HumanBodyBones.LeftUpperLeg},
		{18, HumanBodyBones.LeftLowerLeg},
		{19, HumanBodyBones.LeftFoot},
		{20, HumanBodyBones.LeftToes},
		
		{21, HumanBodyBones.RightUpperLeg},
		{22, HumanBodyBones.RightLowerLeg},
		{23, HumanBodyBones.RightFoot},
		{24, HumanBodyBones.RightToes},
		
		{25, HumanBodyBones.LeftShoulder},
		{26, HumanBodyBones.RightShoulder},
	};
	
	private readonly Dictionary<int, KinectInterop.JointType> boneIndex2JointMap = new Dictionary<int, KinectInterop.JointType>
	{
		{0, KinectInterop.JointType.SpineBase},
		{1, KinectInterop.JointType.SpineMid},
		{2, KinectInterop.JointType.SpineShoulder},
		{3, KinectInterop.JointType.Neck},
		{4, KinectInterop.JointType.Head},
		
		{5, KinectInterop.JointType.ShoulderLeft},
		{6, KinectInterop.JointType.ElbowLeft},
		{7, KinectInterop.JointType.WristLeft},
		{8, KinectInterop.JointType.HandLeft},
		
		{9, KinectInterop.JointType.HandTipLeft},
		{10, KinectInterop.JointType.ThumbLeft},
		
		{11, KinectInterop.JointType.ShoulderRight},
		{12, KinectInterop.JointType.ElbowRight},
		{13, KinectInterop.JointType.WristRight},
		{14, KinectInterop.JointType.HandRight},
		
		{15, KinectInterop.JointType.HandTipRight},
		{16, KinectInterop.JointType.ThumbRight},
		
		{17, KinectInterop.JointType.HipLeft},
		{18, KinectInterop.JointType.KneeLeft},
		{19, KinectInterop.JointType.AnkleLeft},
		{20, KinectInterop.JointType.FootLeft},
		
		{21, KinectInterop.JointType.HipRight},
		{22, KinectInterop.JointType.KneeRight},
		{23, KinectInterop.JointType.AnkleRight},
		{24, KinectInterop.JointType.FootRight},
	};
	
	private readonly Dictionary<int, List<KinectInterop.JointType>> specIndex2JointMap = new Dictionary<int, List<KinectInterop.JointType>>
	{
		{25, new List<KinectInterop.JointType> {KinectInterop.JointType.ShoulderLeft, KinectInterop.JointType.SpineShoulder} },
		{26, new List<KinectInterop.JointType> {KinectInterop.JointType.ShoulderRight, KinectInterop.JointType.SpineShoulder} },
	};
	
	private readonly Dictionary<int, KinectInterop.JointType> boneIndex2MirrorJointMap = new Dictionary<int, KinectInterop.JointType>
	{
		{0, KinectInterop.JointType.SpineBase},
		{1, KinectInterop.JointType.SpineMid},
		{2, KinectInterop.JointType.SpineShoulder},
		{3, KinectInterop.JointType.Neck},
		{4, KinectInterop.JointType.Head},
		
		{5, KinectInterop.JointType.ShoulderRight},
		{6, KinectInterop.JointType.ElbowRight},
		{7, KinectInterop.JointType.WristRight},
		{8, KinectInterop.JointType.HandRight},
		
		{9, KinectInterop.JointType.HandTipRight},
		{10, KinectInterop.JointType.ThumbRight},
		
		{11, KinectInterop.JointType.ShoulderLeft},
		{12, KinectInterop.JointType.ElbowLeft},
		{13, KinectInterop.JointType.WristLeft},
		{14, KinectInterop.JointType.HandLeft},
		
		{15, KinectInterop.JointType.HandTipLeft},
		{16, KinectInterop.JointType.ThumbLeft},
		
		{17, KinectInterop.JointType.HipRight},
		{18, KinectInterop.JointType.KneeRight},
		{19, KinectInterop.JointType.AnkleRight},
		{20, KinectInterop.JointType.FootRight},
		
		{21, KinectInterop.JointType.HipLeft},
		{22, KinectInterop.JointType.KneeLeft},
		{23, KinectInterop.JointType.AnkleLeft},
		{24, KinectInterop.JointType.FootLeft},
	};
	
	private readonly Dictionary<int, List<KinectInterop.JointType>> specIndex2MirrorJointMap = new Dictionary<int, List<KinectInterop.JointType>>
	{
		{25, new List<KinectInterop.JointType> {KinectInterop.JointType.ShoulderRight, KinectInterop.JointType.SpineShoulder} },
		{26, new List<KinectInterop.JointType> {KinectInterop.JointType.ShoulderLeft, KinectInterop.JointType.SpineShoulder} },
	};
	
}

