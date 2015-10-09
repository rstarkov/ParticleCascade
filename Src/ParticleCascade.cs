using System;
using System.Drawing;
using System.Drawing.Imaging;
using RT.Util;
using RT.Util.Geometry;

namespace ParticleCascade
{
    class Field
    {
        private Pixel[] _pixels;
        private Particle[] _particles;
        private int _particleCount; // total number of live particles
        private int _particleLast; // highest index of a live particle. Does not necessarily point to a live particle, but there are no live particles after this index.
        private int _particleFree; // hint at a free (non-live) particle location. This index is not necessarily free, but the search for a free location should start here.
        private int _width;
        private int _height;
        private double _dt;
        private double _time;

        public int ParticleCount { get { return _particleCount; } }

        private Field(int width, int height, double dt = 1)
        {
            _width = width;
            _height = height;
            _dt = dt;
            _pixels = new Pixel[_width * _height];
            _particles = new Particle[1024];
            _particleCount = 0;
            _particleLast = -1;
            _particleFree = 0;
        }

        public static Field InitBreakout1()
        {
            var field = new Field(650, 650, 1);
            for (int x = 0; x < field._width; x++)
            {
                field._pixels[0 * field._width + x].Type = 1;
                field._pixels[(field._height - 1) * field._width + x].Type = 1;
            }
            for (int y = 0; y < field._height; y++)
            {
                field._pixels[y * field._width + 0].Type = 1;
                field._pixels[y * field._width + (field._width - 1)].Type = 1;
            }
            for (int y = 1; y < field._height - 200; y++)
                for (int x = 1; x < field._width - 1; x++)
                    field._pixels[y * field._width + x].Type = 3;
            for (int i = 1; i <= 1; i++)
            {
                int p = field.addParticle();
                field._particles[p].X = field._width / 2;
                field._particles[p].Y = field._height - 10;
                field._particles[p].SetAngleSpeed(5.6, Rnd.NextDouble(0.1, 0.9));
            }
            return field;
        }

        public void Step()
        {
            _time += _dt;
            for (int i = 0; i <= _particleLast; i++)
                if (_particles[i].Live)
                {
                    var newX = _particles[i].X + _particles[i].VX * _dt;
                    var newY = _particles[i].Y + _particles[i].VY * _dt;
                    if (newX < 0 || newY < 0 || newX >= _width || newY >= _height)
                    {
                        deleteParticle(i);
                        continue;
                    }
                    if ((int) newX != (int) _particles[i].X || (int) newY != (int) _particles[i].Y)
                    {
                        // The particle has moved into another pixel
                        int oldPxX = (int) _particles[i].X;
                        int oldPxY = (int) _particles[i].Y;
                        int newPxX = (int) newX;
                        int newPxY = (int) newY;
                        // Particles move no more than 1 pixel at a time, but it could be diagonal. Check all three possible targets before doing any expensive computation.
                        if (_pixels[newPxY * _width + newPxX].IsBounce || _pixels[oldPxY * _width + newPxX].IsBounce || _pixels[newPxY * _width + oldPxX].IsBounce)
                        {
                            var particlePath = new EdgeD(_particles[i].X, _particles[i].Y, newX, newY);
                            var bounce = new bounceResult();

                            if (newPxX > oldPxX) // then check left edge of pixel to the right
                                if (checkBounce(particlePath, newPxX, oldPxY, 2, ref bounce))
                                    goto found;
                            if (newPxX < oldPxX) // then check right edge of pixel to the left
                                if (checkBounce(particlePath, newPxX, oldPxY, 0, ref bounce))
                                    goto found;
                            if (newPxY > oldPxY) // then check top edge of pixel below
                                if (checkBounce(particlePath, oldPxX, newPxY, 3, ref bounce))
                                    goto found;
                            if (newPxY < oldPxY) // then check bottom edge of pixel above
                                if (checkBounce(particlePath, oldPxX, newPxY, 1, ref bounce))
                                    goto found;

                            // No collisions found with the pixels sharing an edge with the old location, but check the diagonally adjacent pixel too
                            if (newPxX != oldPxX && newPxY != oldPxY && _pixels[newPxY * _width + newPxX].IsBounce)
                            {
                                if (checkBounce(particlePath, newPxX, newPxY, 0, ref bounce))
                                    goto found;
                                if (checkBounce(particlePath, newPxX, newPxY, 1, ref bounce))
                                    goto found;
                                if (checkBounce(particlePath, newPxX, newPxY, 2, ref bounce))
                                    goto found;
                                if (checkBounce(particlePath, newPxX, newPxY, 3, ref bounce))
                                    goto found;
                                // The particle has moved into the diagonally adjacent bouncy pixel, so there HAS to be a collision
                                throw new Exception();
                            }

                            goto done; // no collisions found - it's a near miss

                            found: ;

                            newX = bounce.Point.X;
                            newY = bounce.Point.Y;
                            if (bounce.Edge == 0 || bounce.Edge == 2)
                                _particles[i].VX = -_particles[i].VX;
                            else
                                _particles[i].VY = -_particles[i].VY;
                            if (_pixels[bounce.PixelY * _width + bounce.PixelX].IsBreakout)
                            {
                                _pixels[bounce.PixelY * _width + bounce.PixelX].Type = 0;
                                int a = addParticle();
                                _particles[a].X = bounce.Point.X;
                                _particles[a].Y = bounce.Point.Y;
                                _particles[a].SetAngleSpeed(rndQuadrant(new PointD(_particles[i].VX, _particles[i].VY).Theta()), Rnd.NextDouble(0.8, 0.9));
                                _particles[i].Bounces++;
                                if (_particles[i].Bounces >= 3)
                                    deleteParticle(i);
                            }

                            done: ;
                        }
                    }
                    _particles[i].X = newX;
                    _particles[i].Y = newY;
                }

            if (_time % 300 < 10)
            {
                for (int i = 0; i < Math.Max(1, _particleCount / 10 / 10); i++)
                {
                    int a = addParticle();
                    _particles[a].X = _width / 4 + Rnd.NextDouble(-5, 5);
                    _particles[a].Y = _height - 1;
                    _particles[a].SetAngleSpeed(Math.PI + 1.9 + Rnd.NextDouble(-0.05, 0.05), Rnd.NextDouble(0.8, 0.9));
                }
            }
        }

        Random rnd2 = new Random();
        private Color rndColor()
        {
            while (true)
            {
                int r = rnd2.Next(256);
                int g = rnd2.Next(256);
                int b = rnd2.Next(256);
                int cmax = Math.Max(Math.Max(r, g), b);
                if (cmax < 230)
                    continue;
                int cmin = Math.Min(Math.Min(r, g), b);
                if ((cmax - cmin) / (double) cmax < 0.7)
                    continue;
                if (b > 200 && r < 100 & g < 100)
                    continue;
                return Color.FromArgb(r, g, b);
            }
        }

        private double rndQuadrant(double angle)
        {
            angle = angle % (2 * Math.PI);
            if (angle < Math.PI / 2)
                return Rnd.NextDouble(0, Math.PI / 2);
            else if (angle < Math.PI)
                return Rnd.NextDouble(Math.PI / 2, Math.PI);
            else if (angle < 3 * Math.PI / 2)
                return Rnd.NextDouble(Math.PI, 3 * Math.PI / 2);
            else
                return Rnd.NextDouble(3 * Math.PI / 2, 2 * Math.PI);
        }

        private struct bounceResult
        {
            public PointD Point;
            public int Edge;
            public int PixelX, PixelY;
        }

        private bool checkBounce(EdgeD particlePath, int pixelX, int pixelY, int pixelEdge, ref bounceResult result)
        {
            if (!_pixels[pixelY * _width + pixelX].IsBounce)
                return false;
            EdgeD edge;
            switch (pixelEdge)
            {
                case 0:
                    edge = new EdgeD(pixelX + 1, pixelY, pixelX + 1, pixelY + 1); // right edge
                    break;
                case 1:
                    edge = new EdgeD(pixelX, pixelY + 1, pixelX + 1, pixelY + 1); // bottom edge
                    break;
                case 2:
                    edge = new EdgeD(pixelX, pixelY, pixelX, pixelY + 1); // left edge
                    break;
                case 3:
                    edge = new EdgeD(pixelX, pixelY, pixelX + 1, pixelY); // top edge
                    break;
                default:
                    throw new Exception();
            }
            if (!particlePath.IntersectsWith(edge))
                return false;
            result.Point = Intersect.LineWithLine(particlePath, edge);
            result.Edge = pixelEdge;
            result.PixelX = pixelX;
            result.PixelY = pixelY;
            // Ensure that the particle isn't considered to be inside the pixel it bounced off
            if (pixelEdge == 2)
                result.Point.X -= 0.001;
            else if (pixelEdge == 3)
                result.Point.Y -= 0.001;
            return true;
        }

        public Bitmap Draw(Bitmap bmp = null)
        {
            if (bmp == null)
                bmp = new Bitmap(_width, _height, PixelFormat.Format24bppRgb);
            using (var g = Graphics.FromImage(bmp))
                g.Clear(Color.Black);
            for (int y = 0; y < _height; y++)
                for (int x = 0; x < _width; x++)
                    if (_pixels[y * _width + x].Type != 0)
                    {
                        if (_pixels[y * _width + x].IsBreakout)
                            bmp.SetPixel(x, y, Color.Silver);
                        else if (_pixels[y * _width + x].IsBounce)
                            bmp.SetPixel(x, y, Color.DimGray);
                    }
            for (int i = 0; i <= _particleLast; i++)
                if (_particles[i].Live)
                    bmp.SetPixel((int) _particles[i].X, (int) _particles[i].Y, _particles[i].Color);
            return bmp;
        }

        public Bitmap DrawZoomed(int zoom, Bitmap bmp = null)
        {
            if (bmp == null)
                bmp = new Bitmap(_width * zoom, _height * zoom, PixelFormat.Format24bppRgb);
            using (var g = Graphics.FromImage(bmp))
            {
                g.Clear(Color.Black);
                for (int y = 0; y < _height; y++)
                    for (int x = 0; x < _width; x++)
                        if (_pixels[y * _width + x].Type != 0)
                        {
                            if (_pixels[y * _width + x].IsBreakout)
                                g.FillRectangle(Brushes.Silver, x * zoom, y * zoom, zoom, zoom);
                            else if (_pixels[y * _width + x].IsBounce)
                                g.FillRectangle(Brushes.DimGray, x * zoom, y * zoom, zoom, zoom);
                        }
            }
            for (int i = 0; i <= _particleLast; i++)
                if (_particles[i].Live)
                    bmp.SetPixel((int) (_particles[i].X * zoom), (int) (_particles[i].Y * zoom), _particles[i].Color);
            return bmp;
        }

        private void deleteParticle(int index)
        {
            if (!_particles[index].Live)
                throw new Exception();
            _particles[index].Live = false;
            _particleCount--;
            if (index == _particleLast)
                _particleLast--;
        }

        private int addParticle()
        {
            // Scan from ParticleFree onwards
            int result;
            while (_particleFree < _particles.Length && _particles[_particleFree].Live)
                _particleFree++;
            if (_particleFree < _particles.Length)
                result = _particleFree;
            else
            {
                // Didn't find any free cells before the array ended. Scan from start only if there are enough free locations for this to be worth it
                if (_particleCount / (double) _particles.Length < 0.8)
                {
                    _particleFree = 0;
                    while (_particles[_particleFree].Live) // there has to be a non-live one, so no bounds check required
                        _particleFree++;
                    result = _particleFree;
                }
                else
                {
                    // Not worth scanning for a free cell, just add more
                    var newParticles = new Particle[_particles.Length + _particles.Length / 2];
                    Array.Copy(_particles, newParticles, _particles.Length);
                    result = _particles.Length;
                    _particles = newParticles;
                }
            }
            _particles[result].Live = true;
            _particles[result].Bounces = 0;
            _particleCount++;
            _particleLast = Math.Max(_particleLast, result);
            return result;
        }
    }

    struct Pixel
    {
        public byte Type;
        //public byte Val8;
        //public ushort Val16;
        //public uint Val32;

        public bool IsBounce { get { return (Type & 1) != 0; } }
        public bool IsBreakout { get { return (Type & 2) != 0; } }
    }

    struct Particle
    {
        public bool Live;
        public double X, Y, VX, VY;
        public byte Bounces;

        public Color Color { get { return Bounces == 0 ? Color.Lime : Bounces == 1 ? Color.Yellow : Bounces == 2 ? Color.Red : Color.Magenta; } }

        public void SetAngleSpeed(double angle, double speed)
        {
            var vector = new PointD(angle) * speed;
            VX = vector.X;
            VY = vector.Y;
        }
    }
}
