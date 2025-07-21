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
    public GameObject shirt1;
    public GameObject skirt1;
    private TcpListener listener;
    private Thread listenerThread;
    private Queue<Action> mainThreadActions = new Queue<Action>();


    void Start()
    {
        listenerThread = new Thread(Listen);
        listenerThread.IsBackground = true;
        listenerThread.Start();
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

            GameObject go = null; // AvatarController ctrl = null;
            if (packet.name == "shirt1") go = shirt1; // ctrl = shirt1.GetComponent<AvatarController>();
            else if (packet.name == "skirt1") go = skirt1; // ctrl = skirt1.GetComponent<AvatarController>();

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
                // Deactivate all items (expand this list if needed)
                if (shirt1 != null)
                {
                    Debug.Log($"[Action] Deactivating GameObject: shirt1");
                    shirt1.SetActive(false);
                }
                if (skirt1 != null)
                {
                    Debug.Log($"[Action] Deactivating GameObject: skirt1");
                    skirt1.SetActive(false);
                }
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
