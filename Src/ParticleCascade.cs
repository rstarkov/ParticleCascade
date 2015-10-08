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

        private Field(int width, int height)
        {
            _width = width;
            _height = height;
            _pixels = new Pixel[_width * _height];
            _particles = new Particle[1024];
            _particleCount = 0;
            _particleLast = -1;
            _particleFree = 0;
        }

        public static Field InitBreakout1()
        {
            var field = new Field(640, 480);
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
            for (int y = 1; y < 50; y++)
                for (int x = 1; x < field._width - 1; x++)
                    field._pixels[y * field._width + x].Type = 3;
            int p = field.addParticle();
            field._particles[p].X = field._width / 2;
            field._particles[p].Y = 410; //field.Height / 2;
            field._particles[p].SetAngleSpeed(Rnd.NextDouble(Math.PI + 1, 2 * Math.PI - 1), Rnd.NextDouble(0.1, 0.9));
            field._particles[p].Color = Color.Lime;
            return field;
        }

        public void Step()
        {
            for (int i = 0; i <= _particleLast; i++)
                if (_particles[i].Live)
                {
                    var newX = _particles[i].X + _particles[i].VX;
                    var newY = _particles[i].Y + _particles[i].VY;
                    if (newX < 0 || newY < 0 || newX >= _width || newY >= _height)
                    {
                        deleteParticle(i);
                        continue;
                    }
                    if ((int) newX != (int) _particles[i].X || (int) newY != (int) _particles[i].Y)
                    {
                        // The particle has moved into another pixel
                        int newPxX = (int) newX;
                        int newPxY = (int) newY;
                        if (_pixels[newPxY * _width + newPxX].IsBounce)
                        {
                            var particlePath = new EdgeD(_particles[i].X, _particles[i].Y, newX, newY);
                            EdgeD pixelEdge;
                            PointD intersection;
                            int edge;
                            if (particlePath.IntersectsWith(pixelEdge = new EdgeD(newPxX, newPxY, newPxX, newPxY + 1))) // left edge
                            {
                                intersection = Intersect.LineWithLine(particlePath, pixelEdge);
                                edge = 2;
                            }
                            else if (particlePath.IntersectsWith(pixelEdge = new EdgeD(newPxX + 1, newPxY, newPxX + 1, newPxY + 1))) // right edge
                            {
                                intersection = Intersect.LineWithLine(particlePath, pixelEdge);
                                edge = 0;
                            }
                            else if (particlePath.IntersectsWith(pixelEdge = new EdgeD(newPxX, newPxY, newPxX + 1, newPxY))) // top edge
                            {
                                intersection = Intersect.LineWithLine(particlePath, pixelEdge);
                                edge = 3;
                            }
                            else if (particlePath.IntersectsWith(pixelEdge = new EdgeD(newPxX, newPxY + 1, newPxX + 1, newPxY + 1))) // bottom edge
                            {
                                intersection = Intersect.LineWithLine(particlePath, pixelEdge);
                                edge = 1;
                            }
                            else
                                throw new Exception();
                            newX = intersection.X;
                            newY = intersection.Y;
                            if (edge == 0 || edge == 2)
                                _particles[i].VX = -_particles[i].VX;
                            else
                                _particles[i].VY = -_particles[i].VY;

                            if (_pixels[newPxY * _width + newPxX].IsBreakout)
                            {
                                _pixels[newPxY * _width + newPxX].Type = 0;
                                int a = addParticle(); // this new particle may be processed in this Step(), or maybe in the next, depending on where it ends up
                                _particles[a].X = intersection.X;
                                _particles[a].Y = intersection.Y;
                                _particles[a].SetAngleSpeed(edge * Math.PI / 2 + Rnd.NextDouble(-1.3, 1.3), Rnd.NextDouble(0.1, 0.9));
                                _particles[a].Color = _particles[i].Color;
                            }
                        }
                    }
                    _particles[i].X = newX;
                    _particles[i].Y = newY;
                }
        }

        public Bitmap Draw()
        {
            var result = new Bitmap(_width, _height, PixelFormat.Format24bppRgb);
            for (int y = 0; y < _height; y++)
                for (int x = 0; x < _width; x++)
                {
                    var px = _pixels[y * _width + x];
                    var color = Color.Black;
                    if (px.IsBounce)
                        color = Color.Yellow;
                    if (px.IsBreakout)
                        color = Color.Silver;
                    result.SetPixel(x, y, color);
                }
            for (int i = 0; i <= _particleLast; i++)
                if (_particles[i].Live)
                    result.SetPixel((int) _particles[i].X, (int) _particles[i].Y, _particles[i].Color);
            return result;
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
        public Color Color;

        public void SetAngleSpeed(double angle, double speed)
        {
            var vector = new PointD(angle) * speed;
            VX = vector.X;
            VY = vector.Y;
        }
    }
}
