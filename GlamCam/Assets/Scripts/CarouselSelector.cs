using System;
using System.Collections.Generic;
using UnityEngine;

public class CarouselSelector : MonoBehaviour
{
    [Header("Icon to display current item")]
    public MeshRenderer iconRenderer;

    // Assets/Resources/<resourcesFolder>/
    public string resourcesFolder = "Tops";
    public int startIndex = 0;

    [Header("Debug")]
    public bool logChanges = true;

    private readonly List<Texture2D> _items = new();
    private int _index = 0;

    public delegate void OnSelectionChanged(int newIndex, string itemName);
    public event OnSelectionChanged SelectionChanged;

    private static readonly Dictionary<string, string[]> ORDER = new(StringComparer.OrdinalIgnoreCase)
    {
        ["Tops"] = new[]
        {
            "knitted_sweater_01Mesh",
            "elvs_butterflydiscotop1Mesh",
            "vintagetopMesh",
            "Polo_t-shirtMesh",
            "sweater_fishermanMesh",
            "f_lusekoftaMesh",
        },
        ["Bottoms"] = new[]
        {
            "elvs_disco_pant_drufflesMesh",
            "elvs_disco_pant_drufflesMesh",
            "elvs_gored_elephantpant1Mesh",
            "mariaskirtMesh",
            "pleated_skirtelv1Mesh",
        },
        ["Dresses"] = new[]
        {
            "elvs_goddessdress8Mesh",
            "elvs_mini_halterdressMesh",
            "elvs_goddess_dress2Mesh",
            "qipaoMesh",
            "elvs_goddessdress1Mesh",
        },
        ["Fullbodies"] = new[]
        {
            "mars_suitMesh",
            "rescueteam_jacket_maleMesh",
        },
        ["Hats"] = new[]
        {
            "tophat1Mesh",
            "cl_don_quixote_hatMesh",
            "pillow_manMesh",
        },
        ["Accessories"] = new[]
        {
            "tennisracketMesh",
            "suitcaseMesh",
            "bowMesh",
        },
    };

    void Start()
    {
        LoadItems();

        if (_items.Count == 0)
        {
            Debug.LogWarning($"[CarouselSelector] No items loaded for '{resourcesFolder}'.");
            return;
        }

        if (iconRenderer == null)
        {
            Debug.LogWarning("[CarouselSelector] iconRenderer not set.");
            return;
        }

        _index = Mathf.Clamp(startIndex, 0, _items.Count - 1);
        RefreshIcon();
        NotifySelectionChanged();
    }

    public void Next()
    {
        if (_items.Count == 0) return;
        _index = (_index + 1) % _items.Count;
        RefreshIcon();
        NotifySelectionChanged();
    }

    public void Prev()
    {
        if (_items.Count == 0) return;
        _index = (_index - 1 + _items.Count) % _items.Count;
        RefreshIcon();
        NotifySelectionChanged();
    }

    public int GetIndex() => _index;
    public Texture2D GetCurrentTexture() => (_items.Count == 0) ? null : _items[_index];
    public string GetCurrentItemName() => (_items.Count == 0) ? "" : _items[_index].name;

    private void LoadItems()
    {
        _items.Clear();

        // Always include None first if it exists
        AddIfExists("None");

        // Load everything from folder
        var loaded = Resources.LoadAll<Texture2D>(resourcesFolder);
        if (loaded == null || loaded.Length == 0)
        {
            Debug.LogWarning($"[CarouselSelector] No textures found in Assets/Resources/{resourcesFolder}/");
            return;
        }

        var byName = new Dictionary<string, Texture2D>(StringComparer.OrdinalIgnoreCase);
        foreach (var t in loaded)
        {
            if (t != null) byName[t.name] = t;
        }

        // Removes any texture whose name == resourcesFolder
        if (!string.IsNullOrEmpty(resourcesFolder))
            byName.Remove(resourcesFolder);

        byName.Remove("None");

        // Use hardcoded order if defined, else alphabetical
        if (ORDER.TryGetValue(resourcesFolder, out var wantedOrder) && wantedOrder != null && wantedOrder.Length > 0)
        {
            foreach (var name in wantedOrder)
            {
                if (string.IsNullOrWhiteSpace(name)) continue;

                if (byName.TryGetValue(name, out var tex))
                {
                    _items.Add(tex);
                    byName.Remove(name); // so we can append remaining ones
                }
                else if (logChanges)
                {
                    Debug.LogWarning($"[CarouselSelector] '{resourcesFolder}': missing '{name}'");
                }
            }

            // Append any remaining textures (so new assets still show up)
            AppendAlphabetical(byName.Values);
        }
        else
        {
            AppendAlphabetical(byName.Values);
        }

        if (logChanges)
        {
            Debug.Log($"[CarouselSelector] Loaded {_items.Count} items for '{resourcesFolder}' (including None if found).");
        }
    }

    private void AddIfExists(string resourceName)
    {
        var tex = Resources.Load<Texture2D>(resourceName);
        if (tex != null) _items.Add(tex);
        else Debug.LogWarning($"[CarouselSelector] {resourceName}.png not found at Assets/Resources/{resourceName}.png");
    }

    private void AppendAlphabetical(IEnumerable<Texture2D> textures)
    {
        var list = new List<Texture2D>(textures);
        list.Sort((a, b) => string.Compare(a.name, b.name, StringComparison.OrdinalIgnoreCase));
        _items.AddRange(list);
    }

    private void RefreshIcon()
    {
        var tex = _items[_index];
        var mat = iconRenderer.material;

        mat.mainTexture = tex;
        if (mat.HasProperty("_BaseMap")) mat.SetTexture("_BaseMap", tex);

        if (logChanges)
            Debug.Log($"[CarouselSelector] index={_index} texture={tex.name}");
    }

    private void NotifySelectionChanged()
    {
        SelectionChanged?.Invoke(_index, GetCurrentItemName());
    }
}
