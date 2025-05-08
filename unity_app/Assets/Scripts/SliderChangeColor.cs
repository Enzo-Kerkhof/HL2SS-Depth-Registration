// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

using Microsoft.MixedReality.Toolkit.UI;
using UnityEngine;

namespace Microsoft.MixedReality.Toolkit.Examples.Demos
{
    public class SliderChangeColor : MonoBehaviour
    {
        [SerializeField]
        private Renderer TargetRenderer;

        public void OnSliderUpdated(SliderEventData eventData)
        {
            TargetRenderer = GetComponentInChildren<Renderer>();
            if ((TargetRenderer != null) && (TargetRenderer.material != null))
            {
                TargetRenderer.material.SetColor("_WireColor", new Color(eventData.NewValue, eventData.NewValue, eventData.NewValue));
            }
        }

        public void OnSliderUpdatedTransparency(SliderEventData eventData)
        {
            TargetRenderer = GetComponentInChildren<Renderer>();
            if ((TargetRenderer != null) && (TargetRenderer.material != null))
            {
                TargetRenderer.material.SetColor("_Color", new Color(eventData.NewValue, eventData.NewValue, eventData.NewValue, eventData.NewValue));
            }
        }
        public void OnSliderUpdatedWireThickness(SliderEventData eventData)
        {
            TargetRenderer = GetComponentInChildren<Renderer>();
            if ((TargetRenderer != null) && (TargetRenderer.material != null))
            {
                TargetRenderer.material.SetFloat("_WireThickness", eventData.NewValue*800);
            }
        }
    }
}
