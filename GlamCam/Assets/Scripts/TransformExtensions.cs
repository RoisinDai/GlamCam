using UnityEngine;

public static class TransformExtensions
{
  // Finds a child component by name, recursively searching through all children
  // Returns the first child with the specified name, or null if not found
    public static Transform FindDeepChild(this Transform parent, string name)
  {
    foreach (Transform child in parent)
    {
      if (child.name == name)
        return child;

      var result = child.FindDeepChild(name);
      if (result != null)
        return result;
    }
    return null;
  }
}
