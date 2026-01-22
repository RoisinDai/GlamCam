using System;
using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using System.Collections.Generic;
using UnityEngine;

public class ClothingSelectionListener : MonoBehaviour
{
    public int port = 5008;
    public GameObject clothedAvatar;
    private TcpListener listener;
    private Thread listenerThread;
    private Queue<Action> mainThreadActions = new Queue<Action>();
    private readonly Dictionary<string, GameObject> clothingByName = new Dictionary<string, GameObject>();


    void Start()
    {
        if (clothedAvatar == null)
        {
            clothedAvatar = GameObject.Find("clothed_avatar");
            if (clothedAvatar == null)
            {
                Debug.LogWarning("[UIStreamer] clothed_avatar not found in scene. Assign it in the inspector.");
            }
        }

        BuildClothingLookup();
        listenerThread = new Thread(Listen);
        listenerThread.IsBackground = true;
        listenerThread.Start();
    }

    void BuildClothingLookup()
    {
        clothingByName.Clear();
        if (clothedAvatar == null) return;

        var transforms = clothedAvatar.GetComponentsInChildren<Transform>(true);
        foreach (var t in transforms)
        {
            if (t == null || t.gameObject == null) continue;
            if (t.gameObject == clothedAvatar) continue;

            if (!clothingByName.ContainsKey(t.name))
            {
                clothingByName.Add(t.name, t.gameObject);
            }
        }

        Debug.Log($"[UIStreamer] Clothing lookup built with {clothingByName.Count} items.");
    }

    void Listen()
    {
        listener = new TcpListener(IPAddress.Loopback, port);
        listener.Start();
        Debug.Log("[Unity TCP Listener] Started on port " + port);

        while (true)
        {
            using (var client = listener.AcceptTcpClient())
            using (var stream = client.GetStream())
            {
                byte[] buffer = new byte[4096];
                int bytesRead = stream.Read(buffer, 0, buffer.Length);
                if (bytesRead > 0)
                {
                    string json = Encoding.UTF8.GetString(buffer, 0, bytesRead);
                    Debug.Log("[Unity TCP Listener] Got: " + json);

                    // Enqueue a main-thread action to change clothing
                    lock (mainThreadActions)
                    {
                        mainThreadActions.Enqueue(() => HandleClothingPacket(json));
                    }
                }
            }
        }
    }

    void Update()
    {
        // Debug.Log("[Debug] ClothingSelectionListener Update running.");
        lock (mainThreadActions)
        {
            while (mainThreadActions.Count > 0)
            {
                Debug.Log("[Debug] Dequeuing an action.");
                mainThreadActions.Dequeue().Invoke();
            }
        }
    }

    [Serializable]
    public class ClothingPacket
    {
        public string type;
        public string action;
        public string name;
    }

    void HandleClothingPacket(string json)
    {
        try
        {
            Debug.Log($"[Packet] Received JSON: {json}");
            ClothingPacket packet = JsonUtility.FromJson<ClothingPacket>(json);
            if (packet == null)
            {
                Debug.LogWarning("[Packet] Failed to parse packet!");
                return;
            }

            Debug.Log($"[Packet] Parsed: type={packet.type}, action={packet.action}, name={packet.name}");

            GameObject go = null;
            if (!string.IsNullOrEmpty(packet.name) && clothingByName.TryGetValue(packet.name, out var found))
            {
                go = found;
            }

            if (packet.action == "select")
            {
                if (go != null)
                {
                    Debug.Log($"[Action] Activating GameObject: {packet.name}");
                    go.SetActive(true); // Show the activated ClothedBaseAvatar
                    // ctrl.Activate();
                }
                else
                {
                    Debug.LogWarning($"[Action] GameObject '{packet.name}' not found!");
                }
            }
            else if (packet.action == "deselect")
            {
                if (go != null)
                {
                    Debug.Log($"[Action] Deactivating GameObject: {packet.name}");
                    go.SetActive(false);
                }
                else
                {
                    Debug.LogWarning($"[Action] GameObject '{packet.name}' not found!");
                }
            }
            else if (packet.action == "clear")
            {
                foreach (var kvp in clothingByName)
                {
                    if (kvp.Value != null)
                    {
                        kvp.Value.SetActive(false);
                    }
                }
                Debug.Log("[Action] Deactivated all clothing items.");
            }
            else
            {
                Debug.LogWarning($"[Packet] Unknown action: {packet.action}");
            }
        }
        catch (Exception e)
        {
            Debug.LogWarning("Failed to handle clothing packet: " + e.Message);
        }
    }

    void OnApplicationQuit()
    {
        listener?.Stop();
        listenerThread?.Abort();
    }
}
