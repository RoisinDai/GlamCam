using System.IO;
using UnityEditor;
using UnityEngine;

[InitializeOnLoad]
public static class PlayFlagWatcher
{
    // Note: Project root is parent of Assets/
    private static readonly string ProjectRoot =
        Directory.GetParent(Application.dataPath).FullName;

    private static readonly string PlayFlagPath =
        Path.Combine(ProjectRoot, "PLAY.flag");

    private static readonly string StopFlagPath =
        Path.Combine(ProjectRoot, "STOP.flag");

    static PlayFlagWatcher()
    {
        // Runs every editor update tick
        EditorApplication.update += Update;
    }

    private static void Update()
    {
        // STOP: if flag exists, stop play mode (and delete flag)
        if (File.Exists(StopFlagPath))
        {
            File.Delete(StopFlagPath);
            if (EditorApplication.isPlaying)
            {
                EditorApplication.isPlaying = false;
            }
            return; // don't also process PLAY in same tick
        }

        // PLAY: if not playing and play flag exists, start play mode
        if (!EditorApplication.isPlaying && File.Exists(PlayFlagPath))
        {
            File.Delete(PlayFlagPath);
            EditorApplication.isPlaying = true;
        }
    }
}
