using System.Collections.Generic;
using UnityEngine;

public class CarouselSelector : MonoBehaviour
{
    [Header("Icon to display current item")]
    public MeshRenderer iconRenderer;          
    public string resourcesFolder = "Tops";  
    public int startIndex = 0;

    [Header("Debug")]
    public bool logChanges = true;

    private List<Texture2D> _items = new List<Texture2D>();
    private int _index = 0;

    void Start()
    {
        LoadItems();
        if (_items.Count == 0)
        {
            Debug.LogWarning($"[CarouselSelector] No textures found in Resources/{resourcesFolder}");
            return;
        }

        _index = Mathf.Clamp(startIndex, 0, _items.Count - 1);
        RefreshIcon();
    }

    public void Next()
    {
        if (_items.Count == 0) return;
        _index = (_index + 1) % _items.Count;
        RefreshIcon();
    }

    public void Prev()
    {
        if (_items.Count == 0) return;
        _index = (_index - 1 + _items.Count) % _items.Count;
        RefreshIcon();
    }

    public int GetIndex() => _index;
    public Texture2D GetCurrentTexture() => (_items.Count == 0) ? null : _items[_index];

    private void LoadItems()
    {
        _items.Clear();
        var loaded = Resources.LoadAll<Texture2D>(resourcesFolder);
        if (loaded != null) _items.AddRange(loaded);

        // stable ordering by name (so it doesn't shuffle between runs)
        _items.Sort((a, b) => string.Compare(a.name, b.name, System.StringComparison.OrdinalIgnoreCase));
    }

    private void RefreshIcon()
    {
        if (iconRenderer == null)
        {
            Debug.LogWarning("[CarouselSelector] iconRenderer not set.");
            return;
        }

        Texture2D tex = _items[_index];

        // Use renderer.material to avoid editing shared material.
        var mat = iconRenderer.material;

        // Built-in commonly uses mainTexture; URP uses _BaseMap
        mat.mainTexture = tex;
        if (mat.HasProperty("_BaseMap")) mat.SetTexture("_BaseMap", tex);

        if (logChanges)
            Debug.Log($"[CarouselSelector] index={_index} texture={tex.name}");
    }
}