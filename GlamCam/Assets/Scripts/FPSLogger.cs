using UnityEngine;
using System.Collections.Generic;
using System.IO;
using System.Text;

public class FPSLogger : MonoBehaviour
{
    [Tooltip("How often to update the FPS reading (in seconds)")]
    public float updateInterval = 0.5f;

    private float accumulator = 0f;
    private int frames = 0;
    private float timeRemaining;

    [Tooltip("Stores the FPS history for your graph")]
    public List<float> fpsHistory = new List<float>();

    void Start()
    {
        timeRemaining = updateInterval;
    }

    void Update()
    {
        // Track the time and frames
        timeRemaining -= Time.unscaledDeltaTime;
        accumulator += Time.unscaledDeltaTime;
        frames++;

        // Once the interval is reached, calculate the FPS
        if (timeRemaining <= 0.0)
        {
            float currentFps = frames / accumulator;
            
            // Add to your graph data
            fpsHistory.Add(currentFps);

            // Reset variables for the next interval
            timeRemaining = updateInterval;
            accumulator = 0f;
            frames = 0;
        }
    }

    // Triggered automatically when the game closes or the Editor stops playing
    void OnApplicationQuit()
    {
        ExportToCSV();
    }

    private void ExportToCSV()
    {
        // Skip exporting if we didn't record any data
        if (fpsHistory.Count == 0) return;

        StringBuilder csvContent = new StringBuilder();
        
        // Add the header row
        csvContent.AppendLine("Time(s),FPS");

        // Loop through our data and format it
        for (int i = 0; i < fpsHistory.Count; i++)
        {
            float timeElapsed = (i + 1) * updateInterval;
            csvContent.AppendLine($"{timeElapsed:F2},{fpsHistory[i]:F2}");
        }

        // Define the target directory and file name
        string directoryPath = "LogsForAnalysis";
        string fileName = "FPS_Log.csv";
        string filePath = Path.Combine(directoryPath, fileName);

        // Ensure the directory exists before attempting to write the file
        if (!Directory.Exists(directoryPath))
        {
            Directory.CreateDirectory(directoryPath);
        }

        // Write the file to disk
        File.WriteAllText(filePath, csvContent.ToString());

        // Log the path in the Unity Console
        Debug.Log($"<color=green><b>FPS Data successfully exported to:</b></color> {filePath}");
    }
}