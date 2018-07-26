using UnityEngine;
using System.Collections;
using System.Collections.Generic;

public class PresentationScript : MonoBehaviour 
{
	public bool slideChangeWithGestures = true;
	public bool slideChangeWithKeys = true;
	public float spinSpeed = 5;
	
	public bool autoChangeAlfterDelay = false;
	public float slideChangeAfterDelay = 10;
	
	public List<Texture> slideTextures;
	public List<GameObject> horizontalSides;

    // 如果表示多维数据集位于用户的后面(true)或用户的前面(false)
	public bool isBehindUser = false;
	
	private int maxSides = 0;
	private int maxTextures = 0;
	private int side = 0;
	private int tex = 0;
	private bool isSpinning = false;
	private float slideWaitUntil;
	private Quaternion targetRotation;
	
	private GestureListener gestureListener;
	

	
	void Start() 
	{
        // 隐藏鼠标光标
		Cursor.visible = false;

        // 计算最大幻灯片和纹理
		maxSides = horizontalSides.Count;
		maxTextures = slideTextures.Count;

        // 延迟第一张幻灯片
		slideWaitUntil = Time.realtimeSinceStartup + slideChangeAfterDelay;
		
		targetRotation = transform.rotation;
		isSpinning = false;
		
		tex = 0;
		side = 0;
		
		if(horizontalSides[side] && horizontalSides[side].GetComponent<Renderer>())
		{
			horizontalSides[side].GetComponent<Renderer>().material.mainTexture = slideTextures[tex];
		}

        //把动作监听器
		gestureListener = Camera.main.GetComponent<GestureListener>();
	}
	
	void Update() 
	{
        //如果没有用户，不要运行Update()
		KinectManager kinectManager = KinectManager.Instance;
		if(autoChangeAlfterDelay && (!kinectManager || !kinectManager.IsInitialized() || !kinectManager.IsUserDetected()))
			return;
		
		if(!isSpinning)
		{
			if(slideChangeWithKeys)
			{
				if(Input.GetKeyDown(KeyCode.PageDown))
					RotateToNext();
				else if(Input.GetKeyDown(KeyCode.PageUp))
					RotateToPrevious();
			}
			
			if(slideChangeWithGestures && gestureListener)
			{
				if(gestureListener.IsSwipeLeft())
					RotateToNext();
				else if(gestureListener.IsSwipeRight())
					RotateToPrevious();
			}

            // 检查在给定的延迟时间后是否自动切换滑块
			if(autoChangeAlfterDelay && Time.realtimeSinceStartup >= slideWaitUntil)
			{
				RotateToNext();
			}
		}
		else
		{
            // 自旋表示
			transform.rotation = Quaternion.Slerp(transform.rotation, targetRotation, spinSpeed * Time.deltaTime);

            // 检查转换是否达到目标旋转。如果是，停止旋转。
			float deltaTargetX = Mathf.Abs(targetRotation.eulerAngles.x - transform.rotation.eulerAngles.x);
			float deltaTargetY = Mathf.Abs(targetRotation.eulerAngles.y - transform.rotation.eulerAngles.y);
			
			if(deltaTargetX < 1f && deltaTargetY < 1f)
			{
				// delay the slide
				slideWaitUntil = Time.realtimeSinceStartup + slideChangeAfterDelay;
				isSpinning = false;
			}
		}
	}
	
	
	private void RotateToNext()
	{
        // 设置下一个纹理幻灯片
		tex = (tex + 1) % maxTextures;
		
		if(!isBehindUser)
		{
			side = (side + 1) % maxSides;
		}
		else
		{
			if(side <= 0)
				side = maxSides - 1;
			else
				side -= 1;
		}

		if(horizontalSides[side] && horizontalSides[side].GetComponent<Renderer>())
		{
			horizontalSides[side].GetComponent<Renderer>().material.mainTexture = slideTextures[tex];
		}

        // 旋转演示
		float yawRotation = !isBehindUser ? 360f / maxSides : -360f / maxSides;
		Vector3 rotateDegrees = new Vector3(0f, yawRotation, 0f);
		targetRotation *= Quaternion.Euler(rotateDegrees);
		isSpinning = true;
	}
	
	
	private void RotateToPrevious()
	{
        //设置之前的纹理幻灯片
		if(tex <= 0)
			tex = maxTextures - 1;
		else
			tex -= 1;
		
		if(!isBehindUser)
		{
			if(side <= 0)
				side = maxSides - 1;
			else
				side -= 1;
		}
		else
		{
			side = (side + 1) % maxSides;
		}
		
		if(horizontalSides[side] && horizontalSides[side].GetComponent<Renderer>())
		{
			horizontalSides[side].GetComponent<Renderer>().material.mainTexture = slideTextures[tex];
		}

        // 旋转演示
		float yawRotation = !isBehindUser ? -360f / maxSides : 360f / maxSides;
		Vector3 rotateDegrees = new Vector3(0f, yawRotation, 0f);
		targetRotation *= Quaternion.Euler(rotateDegrees);
		isSpinning = true;
	}
	
	
}
