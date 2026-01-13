using System.IO;
using UnityEditor;
using UnityEngine;

[InitializeOnLoad]
public static class PlayFlagWatcher
{
    // This is a file OUTSIDE Assets/, at the project root
    // Unity will only enter play mode when our launcher creates this file
    private static readonly string FlagPath =
        Path.Combine(Directory.GetParent(Application.dataPath).FullName, "PLAY.flag");


    // Register to editor update event
    static PlayFlagWatcher()
    {
        // Runs every editor update tick
        EditorApplication.update += Update;
    }

    private static void Update()
    {
        // If Unity is NOT playing and the flag exists, start Play
        if (!EditorApplication.isPlaying && File.Exists(FlagPath))
        {
            File.Delete(FlagPath); // prevent looping. must explicitly create the file again to re-trigger play
            EditorApplication.isPlaying = true;
        }
    }
}
