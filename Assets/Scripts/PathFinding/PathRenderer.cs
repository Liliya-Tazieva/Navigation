using System.Collections;
using System.Collections.Generic;
using System.Linq;
using Assets.Scripts.Core;
using JetBrains.Annotations;
using UnityEngine;

namespace Assets.Scripts.PathFinding {
    public enum Show {
        Observed = 0,
        Path = 1,
        From = 2,
        To = 3
    }

    public class PathRenderer: MonoBehaviour {
        private class RendererUseInformer {
            public Color DefaultColor;
            public uint UseCount;
        }

        private static Dictionary<Renderer, RendererUseInformer> _defaultColors = new Dictionary<Renderer, RendererUseInformer>();
        private readonly HashSet<Renderer> _usedRenderers = new HashSet<Renderer>();

        public DebugInformationAStar DebugInformation;

        public static void MapRebuild() {
            _defaultColors = new Dictionary<Renderer, RendererUseInformer>();
        }

        [UsedImplicitly]
        private IEnumerator Start() {
            yield return StartCoroutine( "RendererPath" );
        }

        [UsedImplicitly]
        public void OnDestroy() {
            _usedRenderers.ForEach( e => _defaultColors[e].UseCount-- );

            if (_defaultColors != null) {
                
            _defaultColors
                .Where( pair => pair.Value.UseCount == 0 )
					.ForEach( pair => {
						if(pair.Key!=null) {
							pair.Key.material.SetColor( "_Color", pair.Value.DefaultColor );
						}
					} );
            }
        }

        [UsedImplicitly]
        public IEnumerator RendererPath() {
            if (DebugInformation != null) {
                yield return AStarDebug(DebugInformation.From, Show.From);
                foreach (var informer in DebugInformation.Observed) {
                    if (informer.InformerNode != DebugInformation.From && informer.InformerNode != DebugInformation.To) {
                        yield return AStarDebug(informer.InformerNode, Show.Observed);
                    }
                }
                yield return AStarDebug(DebugInformation.To, Show.To);
                foreach (var informer in DebugInformation.FinalPath) {
                    if (informer != DebugInformation.From && informer != DebugInformation.To) {
                        yield return AStarDebug(informer, Show.Path);
                    }
                }
            } else yield return null;
            Destroy(this);
        }

        private IEnumerator AStarDebug( Component informer, Show show ) {
            var component = informer.GetComponent<Renderer>();

            RendererUseInformer rendererInformer;

            if ( !_defaultColors.TryGetValue( component, out rendererInformer ) ) {
                var color = component.material.GetColor( "_Color" );

                rendererInformer = new RendererUseInformer() {
                    DefaultColor = color,
                    UseCount = 1
                };
                _usedRenderers.Add( component );
                _defaultColors.Add( component, rendererInformer );
            } else if ( _usedRenderers.Add( component ) ) {
                rendererInformer.UseCount++;
            }

            if ( show == Show.Observed ) {
                component.material.SetColor( "_Color", Color.yellow );
            } else if ( show == Show.Path ) {
                component.material.SetColor( "_Color", Color.red );
            } else if ( show == Show.From ) {
                component.material.SetColor( "_Color", Color.cyan );
            } else {
                component.material.SetColor( "_Color", Color.magenta );
            }
            yield return new WaitForSeconds( .01f );
        }
    }
}