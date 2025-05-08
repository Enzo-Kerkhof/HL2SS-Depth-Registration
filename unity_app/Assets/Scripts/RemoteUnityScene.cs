
using System;
using System.IO;
using System.Collections.Generic;
using UnityEngine;
using TMPro;
//using Dummiesman;

public class RemoteUnityScene : MonoBehaviour
{
    private Dictionary<int, GameObject> m_remote_objects;
    private bool m_loop;
    private bool m_mode;
    private int m_last_key;

    [Tooltip("Set to BasicMaterial to support semi-transparent primitives.")]
    public Material m_material;

    // GameObject for the model you have in the editor which you want to control through the client
    public GameObject model;
    public GameObject model2;
    public GameObject model3;
    public GameObject model4;
    public GameObject model5;
    public GameObject model6;

    void Start()
    {
        m_remote_objects = new Dictionary<int, GameObject>();
        m_loop = false;
        m_mode = false;
    }

    void Update()
    {
        while (GetMessage() && m_loop);
    }

    bool GetMessage()
    {
        uint command;
        byte[] data;
        if (!hl2ss.PullMessage(out command, out data)) { return false; }
        if (command == 3U)
        {
            hl2ss.PushResult(command, GetModelTransform(command, data));
        }
        else
        {
            hl2ss.PushResult(ProcessMessage(command, data), null);
            hl2ss.AcknowledgeMessage(command);
        }
        return true;
    }

    uint ProcessMessage(uint command, byte[] data)
    {
        uint ret = 0;

        switch (command)
        {
        case   0: ret = MSG_CreatePrimitive(data);   break;
        case   1: ret = MSG_SetActive(data);         break;
        case   2: ret = MSG_SetWorldTransform(data); break;
        case   3: ret = MSG_SetLocalTransform(data); break;
        case   4: ret = MSG_SetColor(data);          break;
        case   5: ret = MSG_SetTexture(data);        break;
        case   6: ret = MSG_CreateText(data);        break;
        case   7: ret = MSG_SetText(data);           break;
        case   8: ret = MSG_LoadMesh(data);          break;

        case  16: ret = MSG_Remove(data);            break;
        case  17: ret = MSG_RemoveAll(data);         break;
        case  18: ret = MSG_BeginDisplayList(data);  break;
        case  19: ret = MSG_EndDisplayList(data);    break;
        case  20: ret = MSG_SetTargetMode(data);     break;
        case ~0U: ret = MSG_Disconnect(data);        break;
        }

        return ret;
    }

    // OK
    uint AddGameObject(GameObject go)
    {
        int key = go.GetInstanceID();
        m_remote_objects.Add(key, go);
        m_last_key = key;

        return (uint)key;
    }

    // OK
    int GetKey(byte[] data)
    {
        return m_mode ? m_last_key : BitConverter.ToInt32(data, 0);
    }

    float[] GetModelTransform(uint command, byte[] data)
    {
        GameObject go;
        if (m_remote_objects.TryGetValue(GetKey(data), out go))
        {
            Matrix4x4 model_TRS = Matrix4x4.TRS(go.transform.position, go.transform.rotation, new Vector3(1.0f, 1.0f, 1.0f));
            float[] model_transform = new float[16];
            {
                model_transform[0] = model_TRS.m00;
                model_transform[1] = model_TRS.m01;
                model_transform[2] = model_TRS.m02;
                model_transform[3] = model_TRS.m03;
                model_transform[4] = model_TRS.m10;
                model_transform[5] = model_TRS.m11;
                model_transform[6] = model_TRS.m12;
                model_transform[7] = model_TRS.m13;
                model_transform[8] = model_TRS.m20;
                model_transform[9] = model_TRS.m21;
                model_transform[10] = model_TRS.m22;
                model_transform[11] = model_TRS.m23;
                model_transform[12] = model_TRS.m30;
                model_transform[13] = model_TRS.m31;
                model_transform[14] = model_TRS.m32;
                model_transform[15] = model_TRS.m33;
            }
            return model_transform;
        }
        return null;
    }

    // OK
    void UnpackTransform(byte[] data, int offset, out Vector3 position, out Quaternion rotation, out Vector3 locscale)
    {
        float[] f = new float[10];
        for (int i = 0; i < f.Length; ++i) { f[i] = BitConverter.ToSingle(data, offset + (i * 4)); }

        position = new Vector3(f[0], f[1], f[2]);
        rotation = new Quaternion(f[3], f[4], f[5], f[6]);
        locscale = new Vector3(f[7], f[8], f[9]);
    }

    // OK
    uint MSG_Remove(byte[] data)
    {
        if (data.Length < 4) { return 0; }

        GameObject go;
        int key = GetKey(data);
        if (!m_remote_objects.TryGetValue(key, out go)) { return 0; }    
        
        m_remote_objects.Remove(key);
        // Destroy(go); // Fixes Unity GameObject InstanceID key loop

        return 1;
    }

    // OK
    uint MSG_RemoveAll(byte[] data)
    {
        foreach (var go in m_remote_objects.Values) { Destroy(go); }
        m_remote_objects.Clear();
        return 1;
    }

    // OK
    uint MSG_BeginDisplayList(byte[] data)
    {
        m_loop = true;
        return 1;
    }

    // OK
    uint MSG_EndDisplayList(byte[] data)
    {
        m_loop = false;
        return 1;
    }

    // OK
    uint MSG_SetTargetMode(byte[] data)
    {
        if (data.Length < 4) { return 0; }
        m_mode = BitConverter.ToUInt32(data, 0) != 0;
        return 1;
    }

    // OK
    uint MSG_Disconnect(byte[] data)
    {
        m_loop = false;
        m_mode = false;
        m_last_key = 0;

        return ~0U;
    }

    // OK
    uint MSG_CreatePrimitive(byte[] data)
    {
        if (data.Length < 4) { return 0; }

        // flags to set for the already created gameobjects alraedy added to scene
        bool isPatient=false;
        bool isPatient2=false;
        bool isPatient3=false;
        bool isPatient4=false;
        bool isPatient5=false;
        bool isPatient6=false;

        PrimitiveType t = PrimitiveType.Quad;
        switch (BitConverter.ToUInt32(data, 0))
        {
        case 0:  isPatient = true;  break;
        case 1:  isPatient2 = true; break;
        case 2:  isPatient3 = true; break;
        case 3:  isPatient4 = true; break;
        case 4:  isPatient5 = true; break;
        case 5:  isPatient6 = true; break;
        default: break;
        }
        // check if flags of scenes gameobjects were set
        if(isPatient)
        {
            return AddGameObject(model);
        }
        if(isPatient2)
        {
            return AddGameObject(model2);
        }
        if(isPatient3)
        {
            return AddGameObject(model3);
        }
        if(isPatient4)
        {
            return AddGameObject(model4);
        }
        if(isPatient5)
        {
            return AddGameObject(model5);
        }
        if(isPatient6)
        {
            return AddGameObject(model6);
        }
        // otherwise create the remote permitive gameobject
        else
        { 
            GameObject go = GameObject.CreatePrimitive(t);
            go.GetComponent<Renderer>().material = m_material; // TODO new Material instead?
            go.SetActive(false);
            return AddGameObject(go);
        }
    }

    // OK
    uint MSG_SetActive(byte[] data)
    {
        if (data.Length < 8) { return 0; }
        
        GameObject go;
        if (!m_remote_objects.TryGetValue(GetKey(data), out go)) { return 0; }

        go.SetActive(BitConverter.ToInt32(data, 4) != 0);

        return 1;
    }

    // OK
    uint MSG_SetWorldTransform(byte[] data)
    {
        if (data.Length < 44) { return 0; }

        GameObject go;
        if (!m_remote_objects.TryGetValue(GetKey(data), out go)) { return 0; }

        Vector3 position;
        Quaternion rotation;
        Vector3 locscale;

        UnpackTransform(data, 4, out position, out rotation, out locscale);

        go.transform.parent = null;
        go.transform.SetPositionAndRotation(position, rotation);
        go.transform.localScale = locscale;

        return 1;
    }

    // OK
    uint MSG_SetLocalTransform(byte[] data)
    {
        if (data.Length < 44) { return 0; }

        GameObject go;
        if (!m_remote_objects.TryGetValue(GetKey(data), out go)) { return 0; }

        go.transform.parent = transform;

        Vector3 position;
        Quaternion rotation;
        Vector3 locscale;

        UnpackTransform(data, 4, out position, out rotation, out locscale);

        Camera cam = gameObject.GetComponent<Camera>();

        var half = locscale / 2;

        var cc = transform.InverseTransformPoint(cam.ScreenToWorldPoint(position));
        var ul = transform.InverseTransformPoint(cam.ScreenToWorldPoint(new Vector3(cc.x - half.x, cc.y - half.y, cc.z)));
        var ur = transform.InverseTransformPoint(cam.ScreenToWorldPoint(new Vector3(cc.x + half.x, cc.y - half.y, cc.z)));
        var bl = transform.InverseTransformPoint(cam.ScreenToWorldPoint(new Vector3(cc.x - half.x, cc.y + half.y, cc.z)));

        var dx = ul - ur;
        var dy = ul - bl;

        go.transform.localPosition = cc;
        go.transform.localRotation = rotation;
        go.transform.localScale = new Vector3(Mathf.Abs(dx.x), Mathf.Abs(dy.y), locscale.z);

        return 1;
    }

    // OK
    uint MSG_SetColor(byte[] data)
    {
        if (data.Length < 20) { return 0; }

        GameObject go;
        if (!m_remote_objects.TryGetValue(GetKey(data), out go)) { return 0; }

        go.GetComponent<Renderer>().material.color = new Color(BitConverter.ToSingle(data, 4), BitConverter.ToSingle(data, 8), BitConverter.ToSingle(data, 12), BitConverter.ToSingle(data, 16));

        return 1;
    }

    // OK
    uint MSG_SetTexture(byte[] data)
    {
        if (data.Length < 4) { return 0; }

        GameObject go;
        if (!m_remote_objects.TryGetValue(GetKey(data), out go)) { return 0; }

        Texture2D tex;
        if (data.Length > 4)
        {
            tex = new Texture2D(2, 2);
            byte[] image = new byte[data.Length - 4];
            Array.Copy(data, 4, image, 0, image.Length);
            tex.LoadImage(image);
        }
        else
        {
            tex = null;
        }

        go.GetComponent<Renderer>().material.mainTexture = tex;

        return 1;
    }

    // OK
    uint MSG_CreateText(byte[] data)
    {
        GameObject go = new GameObject();
        TextMeshPro tmp = go.AddComponent<TextMeshPro>();

        go.SetActive(false);

        tmp.enableWordWrapping = false;
        tmp.autoSizeTextContainer = true;
        tmp.alignment = TextAlignmentOptions.Center;
        tmp.verticalAlignment = VerticalAlignmentOptions.Middle;
        tmp.text = "";

        return AddGameObject(go);
    }

    // OK
    uint MSG_SetText(byte[] data)
    {
        if (data.Length < 24) { return 0; }

        GameObject go;
        if (!m_remote_objects.TryGetValue(GetKey(data), out go)) { return 0; }
        TextMeshPro tmp = go.GetComponent<TextMeshPro>();
        if (tmp == null) { return 0; }

        tmp.fontSize = BitConverter.ToSingle(data, 4);
        tmp.color = new Color(BitConverter.ToSingle(data, 8), BitConverter.ToSingle(data, 12), BitConverter.ToSingle(data, 16), BitConverter.ToSingle(data, 20));

        string str;
        if (data.Length > 24)
        {
            byte[] str_bytes = new byte[data.Length - 24];
            Array.Copy(data, 24, str_bytes, 0, str_bytes.Length);
            try { str = System.Text.Encoding.UTF8.GetString(str_bytes); } catch { return 0; }
        }
        else
        {
            str = "";
        }

        tmp.text = str;

        return 1;
    }

    // OK
    uint MSG_LoadMesh(byte[] data)
    {
        //var stream = new MemoryStream(data);
        //var go = new OBJLoader().Load(stream);

        //return AddGameObject(go);
        return 0;
    }
}
