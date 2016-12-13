using System.Collections.Generic;
using JetBrains.Annotations;
using UnityEngine;

namespace Assets.Scripts.Map {
    [ExecuteInEditMode]
    public class TilesManager : MonoBehaviour {
        [UsedImplicitly]
        public List<Color> Colors;

        [UsedImplicitly]
        public List<GameObject> Prefabs;

        public GameObject GetPrefab(Color color) {
            var index = Colors.IndexOf(color);
            return index == -1 ? null : Prefabs[index];
        }
    }
}
