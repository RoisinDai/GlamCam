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
            "denim_jacketMesh",
            "t-shirt_basic_tuckedMesh",
            "mens_boho_top1elvMesh",
            "f_lusekoftaMesh",
            "Polo_t-shirtMesh",
            "vintagetopMesh",
            "elvs_butterflydiscotop1Mesh",
            "knitted_sweater_01Mesh",
        },
        ["Bottoms"] = new[]
        {
            "mariaskirtMesh",
            "elvs_gored_elephantpant1Mesh",
            "elvs_disco_pant_drufflesMesh",
            "m_trousers_02Mesh",
        },
        ["Dresses"] = new[]
        {
            "princessgownMesh",
            "babydoll_elvruff1aMesh",
            "elvs_sailormoon1Mesh",
            "50sdressMesh",
            "elvs_short_halterdress1Mesh",
            "qipaoMesh",
            "elvs_goddessdress8Mesh",
        },
        ["Fullbodies"] = new[]
        {
            "Laputa_SheetaMesh",
            "coatMesh",
            "uniform_jacketMesh",
            "male_elegantsuit01Mesh",
            "rescueteam_jacket_maleMesh",
            "mars_suitMesh",
        },
        ["Hats"] = new[]
        {
            "harvey_jackolanternMesh",
            "pillow_manMesh"
        },
        ["Accessories"] = new[]
        {
            "elv_fduster1Mesh",
            "multi_bangle1elvMesh",
            "magic_sceptreMesh",
            "electricguitarMesh",
            "suitcaseMesh",
            "bowMesh",
            "tennisracketMesh",
        },
    };

    void Awake()
    {
        LoadItems();
    }
    void Start()
    {
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

    public bool TrySelectByName(string itemName, bool notify = true)
    {
        if (string.IsNullOrEmpty(itemName) || _items.Count == 0) return false;

        for (int i = 0; i < _items.Count; i++)
        {
            var t = _items[i];
            if (t == null) continue;

            if (string.Equals(t.name, itemName, StringComparison.OrdinalIgnoreCase))
            {
                _index = i;
                if (iconRenderer != null) RefreshIcon();
                if (notify) NotifySelectionChanged();
                return true;
            }
        }
        return false;
    }

    // Convenience
    public bool TrySelectNone(bool notify = true)
    {
        return TrySelectByName("None", notify);
    }

    public int GetIndex() => _index;
    public Texture2D GetCurrentTexture() => (_items.Count == 0) ? null : _items[_index];
    public string GetCurrentItemName() => (_items.Count == 0) ? "" : _items[_index].name;

    // exposes the loaded item names for category-scoped lookup
    public List<string> GetLoadedItemNames(bool includeNone = false)
    {
        var names = new List<string>();
        foreach (var t in _items)
        {
            if (t == null) continue;
            if (!includeNone && string.Equals(t.name, "None", StringComparison.OrdinalIgnoreCase)) continue;
            names.Add(t.name);
        }
        return names;
    }

    private void LoadItems()
    {
        _items.Clear();

        // Always include None first if it exists
        AddNoneIfExistsInFolder();

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

    private void AddNoneIfExistsInFolder()
    {
        if (string.IsNullOrEmpty(resourcesFolder)) return;

        string path = $"{resourcesFolder}/None";
        var tex = Resources.Load<Texture2D>(path);

        if (tex != null)
            _items.Add(tex);
        else if (logChanges)
            Debug.LogWarning($"[CarouselSelector] None.png not found at Assets/Resources/{path}.png");
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
