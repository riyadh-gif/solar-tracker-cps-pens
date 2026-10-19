# Susun layar demo (jalankan di PowerShell Windows): panel web di separuh kiri, RViz di separuh kanan, keduanya dibawa ke depan.
# Pakai: powershell -ExecutionPolicy Bypass -File tata_layar_demo.ps1
Add-Type @"
using System; using System.Text; using System.Runtime.InteropServices;
public class D {
 public delegate bool P(IntPtr h, IntPtr l);
 [DllImport("user32.dll")] static extern bool EnumWindows(P p, IntPtr l);
 [DllImport("user32.dll")] static extern int GetWindowText(IntPtr h, StringBuilder s, int n);
 [DllImport("user32.dll")] static extern bool IsWindowVisible(IntPtr h);
 [DllImport("user32.dll")] static extern bool ShowWindow(IntPtr h, int c);
 [DllImport("user32.dll")] static extern bool SetWindowPos(IntPtr h, IntPtr a, int x, int y, int cx, int cy, uint f);
 [DllImport("user32.dll")] static extern bool SetForegroundWindow(IntPtr h);
 public static string Place(string suffixOrPart, int x, int y, int w, int h) {
  string r = "tidak ketemu: " + suffixOrPart;
  EnumWindows((hw, l) => { var s = new StringBuilder(256); GetWindowText(hw, s, 256); var t = s.ToString();
   if (IsWindowVisible(hw) && t.Contains(suffixOrPart) && !t.Contains("Visual Studio Code")) {
    ShowWindow(hw, 9); SetWindowPos(hw, (IntPtr)(-1), x, y, w, h, 0x40); SetWindowPos(hw, (IntPtr)(-2), x, y, w, h, 0x40); SetForegroundWindow(hw); r = "ok: " + t; }
   return true; }, IntPtr.Zero); return r; }
}
"@
Add-Type -AssemblyName System.Windows.Forms
$a = [System.Windows.Forms.Screen]::PrimaryScreen.WorkingArea
$half = [int]($a.Width / 2)
[D]::Place("Panel Kontrol Solar Tracker", $a.X, $a.Y, $half, $a.Height)
[D]::Place("RViz (Ubuntu)", $a.X + $half, $a.Y, $half, $a.Height)
