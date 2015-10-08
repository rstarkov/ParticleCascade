using System.Windows.Forms;
using RT.Util;

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
            for (int i = 0; i < 3; i++)
                _field.Step();
            var bmp = _field.Draw();
            //bmp.Save(@"C:\Temp\ParticleCascade\1\{0:00000}.png".Fmt(_frame++), ImageFormat.Png);
            picture.Image = bmp;
        }
    }
}
