using UnityEngine;

public class ResetButton : MonoBehaviour
{
    [Header("References")]
    public InitializationStateMachine stateMachine;

    [Header("Avatar")]
    public string avatarObjectName = "clothed_avatar";

    public void OnResetPressed()
    {
        // 1) Reset UI / phase state
        if (stateMachine != null)
        {
            stateMachine.ResetToInit();
        }
        else
        {
            Debug.LogWarning("[ResetButton] InitializationStateMachine not assigned.");
        }

        // 2) Reset avatar
        GameObject avatarRoot = GameObject.Find(avatarObjectName);
        if (avatarRoot == null)
        {
            Debug.LogWarning($"[ResetButton] '{avatarObjectName}' not found.");
            return;
        }

        AvatarController avatarController =
            avatarRoot.GetComponentInChildren<AvatarController>();

        if (avatarController != null)
        {
            avatarController.Restart();
        }
        else
        {
            Debug.LogWarning("[ResetButton] AvatarController not found on clothed_avatar.");
        }
    }
}
