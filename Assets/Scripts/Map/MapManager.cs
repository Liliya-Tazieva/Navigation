using System.Collections;
using JetBrains.Annotations;
using UnityEngine;

namespace Assets.Scripts.Map {
    [ExecuteInEditMode]
    public class MapManager : MonoBehaviour {
        [UsedImplicitly]
        public Texture2D Map;

        [UsedImplicitly]
        public TilesManager TilesM;

        public IEnumerator InitializeMap() {
            for (var i = 0; i < Map.height; ++i) {
                for (var j = 0; j < Map.width; ++j) {
                    var color = Map.GetPixel(i, j);
                    var prefab = TilesM.GetPrefab(color);
                    if (prefab == null) {
                        continue;
                    }

                    var position = new Vector3(i*3.0f, 0.0f, j*3.0f);
                    var temp = Instantiate(prefab, position, Quaternion.identity) as GameObject;
                    if (temp != null) {
                        temp.transform.parent = gameObject.transform;
                    }
                    if (j%15 == 0) {
                        yield return null;
                    }
                }
            }
        }

        [UsedImplicitly]
        IEnumerator Start() {
#if UNITY_EDITOR
            if (Application.isPlaying) {
                yield return StartCoroutine("InitializeMap");
            }
#else
                yield return StartCoroutine("InitializeMap");
#endif
        }
    }
}