using UnityEngine;

// Static class for measuring avatar dimensions using bone positions
public static class AvatarMeasurement
{   
    // Get avatar height from head to feet
    public static float GetHeight(GameObject armature)
    {
        Transform headTop = armature.transform.FindDeepChild("mixamorig:HeadTop_End");
        Transform footLeft = armature.transform.FindDeepChild("mixamorig:LeftToeBase");
        Transform footRight = armature.transform.FindDeepChild("mixamorig:RightToeBase");

        if (headTop == null || footLeft == null || footRight == null)
        {
            Debug.LogError("AvatarMeasurement: Could not find bones for height calculation.");
            return 0f;
        }

        float footAvgY = (footLeft.position.y + footRight.position.y) / 2f;
        float height = headTop.position.y - footAvgY;
        
        Debug.Log($"AvatarMeasurement: Height = {height:F3} Unity units");
        return height;
    }
    
    // Get total arm length (shoulder to wrist)
    public static float GetArmLength(GameObject armature)
    {
        Transform leftShoulder = armature.transform.FindDeepChild("mixamorig:LeftShoulder");
        Transform leftArm = armature.transform.FindDeepChild("mixamorig:LeftArm");
        Transform leftForeArm = armature.transform.FindDeepChild("mixamorig:LeftForeArm");

        if (leftShoulder == null || leftArm == null || leftForeArm == null)
        {
            Debug.LogError("AvatarMeasurement: Could not find bones for arm length calculation.");
            return 0f;
        }

        float upperArmLength = Vector3.Distance(leftShoulder.position, leftArm.position);
        float foreArmLength = Vector3.Distance(leftArm.position, leftForeArm.position);
        float totalArmLength = upperArmLength + foreArmLength;
        
        Debug.Log($"AvatarMeasurement: Upper arm = {upperArmLength:F3}, Forearm = {foreArmLength:F3}, Total = {totalArmLength:F3}");
        return totalArmLength;
    }
    
    
    // Get total leg length (hip to foot)
    public static float GetLegLength(GameObject armature)
    {
        Transform leftUpLeg = armature.transform.FindDeepChild("mixamorig:LeftUpLeg");
        Transform leftLeg = armature.transform.FindDeepChild("mixamorig:LeftLeg");
        Transform leftFoot = armature.transform.FindDeepChild("mixamorig:LeftFoot");

        if (leftUpLeg == null || leftLeg == null || leftFoot == null)
        {
            Debug.LogError("AvatarMeasurement: Could not find bones for leg length calculation.");
            return 0f;
        }

        float upperLegLength = Vector3.Distance(leftUpLeg.position, leftLeg.position);
        float lowerLegLength = Vector3.Distance(leftLeg.position, leftFoot.position);
        float totalLegLength = upperLegLength + lowerLegLength;
        
        Debug.Log($"AvatarMeasurement: Upper leg = {upperLegLength:F3}, Lower leg = {lowerLegLength:F3}, Total = {totalLegLength:F3}");
        return totalLegLength;
    }
    
} 