using System.Drawing;
using System.Drawing.Imaging;
using System.Windows.Forms;
using RT.Util;
using RT.Util.ExtensionMethods;

namespace ParticleCascade
{
    public partial class MainForm : Form
    {
        private Field _field;
        private int _frame = 0;

        public MainForm()
        {
            InitializeComponent();
            Rnd.Reset(7923);
            _field = Field.InitBreakout1();
        }

        private void timer_Tick(object sender, System.EventArgs e)
        {
                _field.Step();
            //var bmp = _field.DrawZoomed(10, picture.Image as Bitmap);
            var bmp = _field.Draw(picture.Image as Bitmap);
            //bmp.Save(@"C:\Temp\ParticleCascade\8\{0:00000}.png".Fmt(_frame++), ImageFormat.Png);
            picture.Image = bmp;
        }
    }
}
