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
    // ��Ա��ָ����ģ�ʹ���Ķ�����Ĭ��0(��һλ)
	public int playerIndex = 0;

    // ���н�ɫ(������)������Bool��������Ĭ�ϵĴ���
	public bool mirroredMovement = false;

    // Bool���������Ƿ���������ֱ�˶�
	public bool verticalMovement = false;

    // ��������ڳ������ƶ����ٶȡ�
	private int moveRate = 1;

    // Slerp�⻬����
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

    // ���Ҫ�ڿռ�����תģ�ͣ�����Ҫһ��������
	private GameObject offsetNode;

    // �����й�ͷ���̶�ס��������ʼ�����ʼ����ͬ�Ĵ�С��
	private Transform[] bones;

    // ��Kinect��ʼ����ʱ��������ת��
    private Quaternion[] initialRotations;
	private Quaternion[] initialLocalRotations;

    // Kinect׷�ٿ�ʼʱ�����ķ���
    //private Vector3[] initialDirections;

    // �任�ĳ�ʼλ�ú���ת
	private Vector3 initialPosition;
	private Quaternion initialRotation;

    // У���ַ�λ�õ�ƫ����������
	private bool OffsetCalibrated = false;
	private float XOffset, YOffset, ZOffset;
	//private Quaternion originalRotation;
	
	// private instance of the KinectManager
	private KinectManager kinectManager;


    // ת���������������ܣ���Ϊ��ÿ�ε���ת��ʱ��Unity�������GetComponent< transform >()
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
        // ���˫��ʼ
		if(bones != null)
			return;

        // init��ͷ����
		bones = new Transform[27];

        // �����ĳ�ʼת���ͷ���
		initialRotations = new Quaternion[bones.Length];
		initialLocalRotations = new Quaternion[bones.Length];
		//initialDirections = new Vector3[bones.Length];

        //��OffsetNode��Ϊģ��ת���ĸ��ڵ㡣ֻ�ǲ�Ҫÿ�ζ���ô��
		offsetNode = new GameObject(name + "Ctrl") { layer = transform.gameObject.layer, tag = transform.gameObject.tag };
		offsetNode.transform.position = transform.position;
		offsetNode.transform.rotation = transform.rotation;

		transform.parent = offsetNode.transform;
		transform.localPosition = Vector3.zero;
		transform.localRotation = Quaternion.identity;
		
		bodyRoot = transform;

        // ������ӳ�䵽Kinect׷�ٵ�
		MapBones();

        // �õ���ʼ����ת
		GetInitialRotations();
	}

    // ����ÿһ֡��ͷ��
    public void UpdateAvatar(Int64 UserID)
    {	
		if(!transform.gameObject.activeInHierarchy) 
			return;

        // �õ�KinectManagerʵ��
		if(kinectManager == null)
		{
			kinectManager = KinectManager.Instance;
		}

        // �������ƶ�������Kinectλ�á�
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
                // ����Ĺ�ͷ(����)
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

    // �ѹ�ͷ�̶��������λ�ú�ת����
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

        // ����ÿһ�鱻����Ĺ�ͷ������Ϊ��ʼλ�á�
        for (int pass = 0; pass < 2; pass++)  // 2������Ϊ���������
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

        // �ָ�ƫ��λ�ú���ת��
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

    // ����ҳɹ�У׼ʱ���á�
	public void SuccessfulCalibration(Int64 userId)
	{
        // ����ģ��λ��
		if(offsetNode != null)
		{
			offsetNode.transform.rotation = initialRotation;
		}
		else
		{
			transform.rotation = initialRotation;
		}

        // ��ϧ��λ��ƫ��
		OffsetCalibrated = false;
	}

    // ��kinect׷�ٵ���ת����Ӧ�õ��ؽ��ϡ�
	void TransformBone(Int64 userId, KinectInterop.JointType joint, int boneIndex, bool flip)
    {
		Transform boneTransform = bones[boneIndex];
		if(boneTransform == null || kinectManager == null)
			return;
		
		int iJoint = (int)joint;
		if(iJoint < 0 || !kinectManager.IsJointTracked(userId, iJoint))
			return;

        // �õ�Kinect���϶�λ
		Quaternion jointRotation = kinectManager.GetJointOrientation(userId, iJoint, flip);
		if(jointRotation == Quaternion.identity)
			return;

        // ƽ�ȹ��ɵ��µ���ת
		Quaternion newRotation = Kinect2AvatarRot(jointRotation, boneIndex);

		if(smoothFactor != 0f)
        	boneTransform.rotation = Quaternion.Slerp(boneTransform.rotation, newRotation, smoothFactor * Time.deltaTime);
		else
			boneTransform.rotation = newRotation;
	}

    // ��kinect׷�ٵ���תӦ�õ�һ������Ĺؽ���
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
            // ƽ�ȹ��ɵ��µ���ת
			Quaternion newRotation = Kinect2AvatarRot(jointRotation, boneIndex);
			
			if(smoothFactor != 0f)
				boneTransform.rotation = Quaternion.Slerp(boneTransform.rotation, newRotation, smoothFactor * Time.deltaTime);
			else
				boneTransform.rotation = newRotation;
		}
		
	}

    // ��3D�ռ��ƶ������������û������ٵ�λ�ò�Ӧ�õ�root��
	void MoveAvatar(Int64 UserID)
	{
		if(!kinectManager || !kinectManager.IsJointTracked(UserID, (int)KinectInterop.JointType.SpineBase))
			return;

        // ��������λ�ò���������
		Vector3 trans = kinectManager.GetUserPosition(UserID);

        // ����������ǵ�һ���ƶ�ͷ������ƫ�����������������
		if (!OffsetCalibrated)
		{
			OffsetCalibrated = true;
			
			XOffset = !mirroredMovement ? trans.x * moveRate : -trans.x * moveRate;
			YOffset = trans.y * moveRate;
			ZOffset = -trans.z * moveRate;
		}

        // ˳���ع��ɵ��µĸ�λ
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

    // ���Ҫӳ��Ĺ�ͷ�Ѿ����������ù�ͷӳ�䵽ģ�͡�
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

    // ��׽�����������ת
	void GetInitialRotations()
	{
        // �����������ת
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

        // �ָ��������ת
		if(offsetNode != null)
		{
			offsetNode.transform.rotation = initialRotation;
		}
		else
		{
			transform.rotation = initialRotation;
		}
	}

    //��kinect�ؽ���תת��Ϊavatar�ؽ���ת��ȡ���ڹؽڳ�ʼ��ת��ƫ����ת
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

    // ��Kinectλ��ת��Ϊavatar�Ǽ�λ�ã�ȡ���ڳ�ʼλ�á�������ƶ��ٶ�
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

    //�ֵ���Լ��ٹ����Ĵ���
    //���ǹ����˶��ؽڵ�����ӳ��ĺ����������
    //��ͬ���ĳ�ʼʵ�֣����������dictionary is
    //�׹�������Korchun(korchoon@gmail.com)���ǳ���л����һ�!
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

