// Put in Assets/Editor/AvatarLimbMeasurer.cs — delete after measuring
using UnityEngine;
using UnityEditor;

public class AvatarLimbMeasurer : EditorWindow
{
    public GameObject avatar;
    private SkinnedMeshRenderer[] _foundRenderers;
    private string[] _rendererNames;
    private int _selectedRendererIndex = 0;

    [MenuItem("Tools/Measure Avatar Limbs")]
    static void Open() => GetWindow<AvatarLimbMeasurer>();

    void OnGUI()
    {
        EditorGUI.BeginChangeCheck();
        avatar = (GameObject)EditorGUILayout.ObjectField("Avatar", avatar, typeof(GameObject), true);
        if (EditorGUI.EndChangeCheck() && avatar != null)
        {
            // When avatar changes, discover all SkinnedMeshRenderers so the user can pick one
            _foundRenderers = avatar.GetComponentsInChildren<SkinnedMeshRenderer>(true);
            _rendererNames = new string[_foundRenderers.Length];
            for (int i = 0; i < _foundRenderers.Length; i++)
            {
                var mesh = _foundRenderers[i].sharedMesh;
                string meshName = mesh != null ? mesh.name : "(no mesh)";
                _rendererNames[i] = $"{_foundRenderers[i].gameObject.name}  [{meshName}, {(mesh != null ? mesh.vertexCount : 0)} verts]";
            }
            _selectedRendererIndex = 0;
        }

        if (avatar == null) return;

        // Show dropdown to pick which mesh to measure (body vs. clothing)
        if (_rendererNames != null && _rendererNames.Length > 0)
        {
            EditorGUILayout.Space();
            EditorGUILayout.LabelField("Pick the BASE BODY mesh (not clothing):", EditorStyles.boldLabel);
            _selectedRendererIndex = EditorGUILayout.Popup("Mesh to measure", _selectedRendererIndex, _rendererNames);
        }

        EditorGUILayout.Space();
        if (!GUILayout.Button("Measure")) return;

        var animator = avatar.GetComponent<Animator>();
        if (animator == null) { Debug.LogError("No Animator found"); return; }

        if (_foundRenderers == null || _foundRenderers.Length == 0)
        {
            Debug.LogError("No SkinnedMeshRenderers found on avatar");
            return;
        }

        SkinnedMeshRenderer targetRenderer = _foundRenderers[_selectedRendererIndex];
        Debug.Log($"=== Measuring using mesh: {targetRenderer.gameObject.name} ===");

        // Height
        var head = animator.GetBoneTransform(HumanBodyBones.Head);
        var lFoot = animator.GetBoneTransform(HumanBodyBones.LeftFoot);
        var rFoot = animator.GetBoneTransform(HumanBodyBones.RightFoot);
        float height = head.position.y - (lFoot.position.y + rFoot.position.y) / 2f;
        Debug.Log($"Avatar Height: {height:F4} units ({height * 100f:F2} cm)");

        // Measure each limb — only using the selected mesh
        MeasureLimb(animator, targetRenderer, HumanBodyBones.LeftUpperArm, HumanBodyBones.LeftLowerArm, "Upper Arm", height);
        MeasureLimb(animator, targetRenderer, HumanBodyBones.LeftLowerArm, HumanBodyBones.LeftHand, "Forearm", height);
        MeasureLimb(animator, targetRenderer, HumanBodyBones.LeftUpperLeg, HumanBodyBones.LeftLowerLeg, "Thigh", height);
        MeasureLimb(animator, targetRenderer, HumanBodyBones.LeftLowerLeg, HumanBodyBones.LeftFoot, "Calf", height);

        // Measure torso (SpineMid) width using the Spine bone
        MeasureTorsoWidth(animator, targetRenderer, height);
    }

    /// <summary>
    /// Measures the torso width at the Spine bone (corresponds to Kinect SpineMid).
    /// Uses bone-weighted vertices at the Spine bone's Y level and measures the X-axis extent.
    /// </summary>
    void MeasureTorsoWidth(Animator anim, SkinnedMeshRenderer smr, float avatarHeight)
    {
        // Use the Spine bone (corresponds to SpineMid in Kinect)
        Transform spineT = anim.GetBoneTransform(HumanBodyBones.Spine);
        if (spineT == null) { Debug.LogWarning("Missing Spine bone for torso measurement"); return; }

        // Find the spine bone index in the mesh
        Transform[] meshBones = smr.bones;
        // Collect indices for spine-related bones (Hips, Spine, Chest) to capture torso vertices
        var spineBoneIndices = new System.Collections.Generic.HashSet<int>();
        HumanBodyBones[] torsoBonesEnum = { HumanBodyBones.Hips, HumanBodyBones.Spine, HumanBodyBones.Chest, HumanBodyBones.UpperChest };
        foreach (var tb in torsoBonesEnum)
        {
            Transform t = anim.GetBoneTransform(tb);
            if (t == null) continue;
            for (int i = 0; i < meshBones.Length; i++)
            {
                if (meshBones[i] == t) { spineBoneIndices.Add(i); break; }
            }
        }

        if (spineBoneIndices.Count == 0)
        {
            Debug.LogWarning("Torso: No spine bones found in SkinnedMeshRenderer bone list.");
            return;
        }

        // Bake mesh
        Mesh bakedMesh = new Mesh();
        smr.BakeMesh(bakedMesh);
        Vector3[] vertices = bakedMesh.vertices;
        var boneWeights = smr.sharedMesh.boneWeights;

        // Get the Y-level of the Spine bone (SpineMid equivalent)
        float spineY = spineT.position.y;

        // Collect X coordinates of torso vertices near the Spine's Y level
        float yTolerance = avatarHeight * 0.05f; // within 5% of height from spine level
        float minWeight = 0.2f;

        float minX = float.MaxValue;
        float maxX = float.MinValue;
        int vertCount = 0;

        for (int i = 0; i < vertices.Length; i++)
        {
            BoneWeight w = boneWeights[i];
            bool belongsToTorso =
                (spineBoneIndices.Contains(w.boneIndex0) && w.weight0 >= minWeight) ||
                (spineBoneIndices.Contains(w.boneIndex1) && w.weight1 >= minWeight) ||
                (spineBoneIndices.Contains(w.boneIndex2) && w.weight2 >= minWeight) ||
                (spineBoneIndices.Contains(w.boneIndex3) && w.weight3 >= minWeight);

            if (!belongsToTorso) continue;

            Vector3 worldVert = smr.transform.TransformPoint(vertices[i]);

            // Only vertices near the Spine bone's Y level
            if (Mathf.Abs(worldVert.y - spineY) <= yTolerance)
            {
                if (worldVert.x < minX) minX = worldVert.x;
                if (worldVert.x > maxX) maxX = worldVert.x;
                vertCount++;
            }
        }
        DestroyImmediate(bakedMesh);

        if (vertCount < 2)
        {
            Debug.LogWarning($"Torso: Not enough vertices found near Spine Y level ({vertCount} verts).");
            return;
        }

        float torsoWidth = maxX - minX;
        float ratio = torsoWidth / avatarHeight;

        Debug.Log($"Torso (SpineMid) width: {torsoWidth:F4} units ({torsoWidth * 100f:F2} cm), " +
                  $"ratio = {ratio:F4} (width/height)  [AVATAR_WIDTH_HEIGHT_RATIO]  [verts sampled: {vertCount}]");
    }

    void MeasureLimb(Animator anim, SkinnedMeshRenderer smr, HumanBodyBones bone, HumanBodyBones childBone, string label, float avatarHeight)
    {
        var boneT = anim.GetBoneTransform(bone);
        var childT = anim.GetBoneTransform(childBone);
        if (boneT == null || childT == null) { Debug.LogWarning($"Missing bone: {label}"); return; }

        // Find the bone index in the SkinnedMeshRenderer's bone array
        // This lets us use bone weights to filter vertices that actually belong to this bone
        Transform[] meshBones = smr.bones;
        int boneIndex = -1;
        for (int i = 0; i < meshBones.Length; i++)
        {
            if (meshBones[i] == boneT)
            {
                boneIndex = i;
                break;
            }
        }

        if (boneIndex < 0)
        {
            Debug.LogWarning($"{label}: Bone '{boneT.name}' not found in SkinnedMeshRenderer bone list. Cannot measure.");
            return;
        }

        // Midpoint of the bone segment
        Vector3 midpoint = (boneT.position + childT.position) / 2f;

        // Bone direction (length axis)
        Vector3 boneDir = (childT.position - boneT.position).normalized;
        float boneLen = Vector3.Distance(boneT.position, childT.position);

        // Bake the selected mesh and sample vertices
        Mesh sharedMesh = smr.sharedMesh;
        Mesh bakedMesh = new Mesh();
        smr.BakeMesh(bakedMesh);
        Vector3[] vertices = bakedMesh.vertices;

        // Get bone weights to filter vertices that belong to this bone
        var boneWeights = sharedMesh.boneWeights;
        float minWeight = 0.3f; // Vertex must be at least 30% weighted to this bone

        float maxWidth = 0f;
        int vertexCount = 0;

        for (int i = 0; i < vertices.Length; i++)
        {
            // Check if this vertex is weighted to our target bone
            BoneWeight w = boneWeights[i];
            bool belongsToBone =
                (w.boneIndex0 == boneIndex && w.weight0 >= minWeight) ||
                (w.boneIndex1 == boneIndex && w.weight1 >= minWeight) ||
                (w.boneIndex2 == boneIndex && w.weight2 >= minWeight) ||
                (w.boneIndex3 == boneIndex && w.weight3 >= minWeight);

            if (!belongsToBone) continue;

            Vector3 worldVert = smr.transform.TransformPoint(vertices[i]);

            // Project onto bone axis to find distance along bone
            float projAlongBone = Vector3.Dot(worldVert - boneT.position, boneDir);

            // Only consider vertices in the middle 40% of the bone segment
            if (projAlongBone > boneLen * 0.3f && projAlongBone < boneLen * 0.7f)
            {
                // Perpendicular distance from bone axis = cross-section radius
                Vector3 toVert = worldVert - midpoint;
                Vector3 perpendicular = toVert - Vector3.Dot(toVert, boneDir) * boneDir;
                float dist = perpendicular.magnitude;
                if (dist > maxWidth) maxWidth = dist;
                vertexCount++;
            }
        }
        DestroyImmediate(bakedMesh);

        // maxWidth is the radius, diameter = 2 * radius
        float diameter = maxWidth * 2f;
        float ratio = diameter / avatarHeight;

        Debug.Log($"{label}: width = {diameter:F4} units ({diameter * 100f:F2} cm), " +
                  $"ratio = {ratio:F4} (width/height)  [bone: {boneT.name}, idx: {boneIndex}, verts sampled: {vertexCount}]");
    }
}