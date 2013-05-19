classdef BotLCMGLClient < handle
% intended to be exactly a matlab implementation of the lcmgl.c in
% bot_lcmgl_client

  methods
    function lcmgl = BotLCMGLClient(name)
      if ~exist('bot_lcmgl.data_t','class')
        error('can''t find bot_lcmgl.data_t.  for me, i put \n\t%s\n into my startup.m','javaaddpath(''/Users/russt/drc/software/build/share/java/bot2-lcmgl.jar'');');
      end
      
      lcmgl.lcm = lcm.lcm.LCM.getSingleton();
      lcmgl.channel = ['LCMGL_',name];
      
      lcmgl.data_t = bot_lcmgl.data_t();
      lcmgl.data_t.name = name;
      lcmgl.data_t.scene = now;
      lcmgl.data_t.sequence = 0;
      lcmgl.data_t.datalen = 0;
      lcmgl.data_t.data = zeros(1024*1024,1);
    end
    
    function bot_lcmgl_switch_buffer(lcmgl)
      % Transmits all queued operations.
      lcmgl.lcm.publish(lcmgl.channel,lcmgl.data_t);

      lcmgl.data_t.sequence = 0;
      lcmgl.data_t.datalen = 0;
      lcmgl.data_t.scene = lcmgl.data_t.scene+1;
    end
    
    function bot_lcmgl_destroy(lcmgl)
      % not required for matlab version, but left in for compatibility
    end
    
  end
  
  methods
    %% ================ OpenGL functions =========== 

% void bot_lcmgl_begin(bot_lcmgl_t *lcmgl, unsigned int mode)
% {
%     bot_lcmgl_encode_u8(lcmgl, BOT_LCMGL_BEGIN);
%     bot_lcmgl_encode_u32(lcmgl, mode);
% }
% 
% void bot_lcmgl_end(bot_lcmgl_t *lcmgl)
% {
%     bot_lcmgl_encode_u8(lcmgl, BOT_LCMGL_END);
% }
% 
% void bot_lcmgl_vertex2d(bot_lcmgl_t *lcmgl, double v0, double v1)
% {
%     bot_lcmgl_encode_u8(lcmgl, BOT_LCMGL_VERTEX2D);
%     assert (isfinite (v0) && isfinite (v1));
% 
%     bot_lcmgl_encode_double(lcmgl, v0);
%     bot_lcmgl_encode_double(lcmgl, v1);
% }
% 
% void bot_lcmgl_vertex2f(bot_lcmgl_t *lcmgl, float v0, float v1)
% {
%     bot_lcmgl_encode_u8(lcmgl, BOT_LCMGL_VERTEX2F);
%     assert (isfinite (v0) && isfinite (v1));
% 
%     bot_lcmgl_encode_float(lcmgl, v0);
%     bot_lcmgl_encode_float(lcmgl, v1);
% }
% 
% void bot_lcmgl_vertex3f(bot_lcmgl_t *lcmgl, float v0, float v1, float v2)
% {
%     bot_lcmgl_encode_u8(lcmgl, BOT_LCMGL_VERTEX3F);
%     assert (isfinite (v0) && isfinite (v1) && isfinite (v2));
% 
%     bot_lcmgl_encode_float(lcmgl, v0);
%     bot_lcmgl_encode_float(lcmgl, v1);
%     bot_lcmgl_encode_float(lcmgl, v2);
% }
% 
% void bot_lcmgl_vertex3d(bot_lcmgl_t *lcmgl, double v0, double v1, double v2)
% {
%     bot_lcmgl_encode_u8(lcmgl, BOT_LCMGL_VERTEX3D);
%     assert (isfinite (v0) && isfinite (v1) && isfinite (v2));
% 
%     bot_lcmgl_encode_double(lcmgl, v0);
%     bot_lcmgl_encode_double(lcmgl, v1);
%     bot_lcmgl_encode_double(lcmgl, v2);
% }
% 
% void bot_lcmgl_normal3f(bot_lcmgl_t *lcmgl, float v0, float v1, float v2)
% {
%     bot_lcmgl_encode_u8(lcmgl, BOT_LCMGL_NORMAL3F);
%     assert (isfinite (v0) && isfinite (v1) && isfinite (v2));
%     bot_lcmgl_encode_float(lcmgl, v0);
%     bot_lcmgl_encode_float(lcmgl, v1);
%     bot_lcmgl_encode_float(lcmgl, v2);
% }
% 
% void bot_lcmgl_scalef(bot_lcmgl_t *lcmgl, float v0, float v1, float v2)
% {
%     bot_lcmgl_encode_u8(lcmgl, BOT_LCMGL_SCALEF);
%     assert (isfinite (v0) && isfinite (v1) && isfinite (v2));
%     bot_lcmgl_encode_float(lcmgl, v0);
%     bot_lcmgl_encode_float(lcmgl, v1);
%     bot_lcmgl_encode_float(lcmgl, v2);
% }
% 
% 
% void bot_lcmgl_translated(bot_lcmgl_t *lcmgl, double v0, double v1, double v2)
% {
%     bot_lcmgl_encode_u8(lcmgl, BOT_LCMGL_TRANSLATED);
%     assert (isfinite (v0) && isfinite (v1) && isfinite (v2));
% 
%     bot_lcmgl_encode_double(lcmgl, v0);
%     bot_lcmgl_encode_double(lcmgl, v1);
%     bot_lcmgl_encode_double(lcmgl, v2);
% }
% 
% void bot_lcmgl_rotated(bot_lcmgl_t *lcmgl, double angle, double x, double y, double z)
% {
%     bot_lcmgl_encode_u8(lcmgl, BOT_LCMGL_ROTATED);
% 
%     bot_lcmgl_encode_double(lcmgl, angle);
%     bot_lcmgl_encode_double(lcmgl, x);
%     bot_lcmgl_encode_double(lcmgl, y);
%     bot_lcmgl_encode_double(lcmgl, z);
% }
% 
% void bot_lcmgl_push_matrix (bot_lcmgl_t * lcmgl)
% {
%     bot_lcmgl_encode_u8 (lcmgl, BOT_LCMGL_PUSH_MATRIX);
% }
% 
% void bot_lcmgl_pop_matrix (bot_lcmgl_t * lcmgl)
% {
%     bot_lcmgl_encode_u8 (lcmgl, BOT_LCMGL_POP_MATRIX);
% }
% 
% void bot_lcmgl_mult_matrixf(bot_lcmgl_t *lcmgl, const float m[16])
% {
%     bot_lcmgl_encode_u8(lcmgl, BOT_LCMGL_MULT_MATRIXF);
% 
%     for (int i = 0; i < 16; i++)
%         bot_lcmgl_encode_float(lcmgl, m[i]);
% }
% 
% void bot_lcmgl_mult_matrixd(bot_lcmgl_t *lcmgl, const double m[16])
% {
%     bot_lcmgl_encode_u8(lcmgl, BOT_LCMGL_MULT_MATRIXD);
% 
%     for (int i = 0; i < 16; i++)
%         bot_lcmgl_encode_double(lcmgl, m[i]);
% }
% 
% 
% void bot_lcmgl_load_identity(bot_lcmgl_t *lcmgl)
% {
%     bot_lcmgl_encode_u8(lcmgl, BOT_LCMGL_LOAD_IDENTITY);
% }
% 
% void bot_lcmgl_matrix_mode(bot_lcmgl_t * lcmgl, unsigned int mode){
%   bot_lcmgl_encode_u8(lcmgl, BOT_LCMGL_MATRIX_MODE);
%   bot_lcmgl_encode_u32(lcmgl, mode);
% }
% 
% 
% void bot_lcmgl_ortho(bot_lcmgl_t *lcmgl,
%     double        left,    double        right,
%     double        bottom,  double        top,
%     double        nearVal, double        farVal){
%   bot_lcmgl_encode_u8(lcmgl, BOT_LCMGL_ORTHO);
%   bot_lcmgl_encode_double(lcmgl, left);
%   bot_lcmgl_encode_double(lcmgl, right);
%   bot_lcmgl_encode_double(lcmgl, bottom);
%   bot_lcmgl_encode_double(lcmgl, top);
%   bot_lcmgl_encode_double(lcmgl, nearVal);
%   bot_lcmgl_encode_double(lcmgl, farVal);
% }
% 
% 
% 
    function bot_lcmgl_color3f(lcmgl, v0, v1, v2)
      bot_lcmgl_encode_u8(lcmgl, lcmgl.BOT_LCMGL_COLOR3F);
      assert (isfinite (v0) && isfinite (v1) && isfinite (v2));
      bot_lcmgl_encode_float(lcmgl, v0);
      bot_lcmgl_encode_float(lcmgl, v1);
      bot_lcmgl_encode_float(lcmgl, v2);
    end
% 
% void bot_lcmgl_color4f(bot_lcmgl_t *lcmgl, float v0, float v1, float v2, float v3)
% {
%     bot_lcmgl_encode_u8(lcmgl, BOT_LCMGL_COLOR4F);
%     bot_lcmgl_encode_float(lcmgl, v0);
%     bot_lcmgl_encode_float(lcmgl, v1);
%     bot_lcmgl_encode_float(lcmgl, v2);
%     bot_lcmgl_encode_float(lcmgl, v3);
% }
% 
% void bot_lcmgl_point_size(bot_lcmgl_t *lcmgl, float v)
% {
%     bot_lcmgl_encode_u8(lcmgl, BOT_LCMGL_POINTSIZE);
%     bot_lcmgl_encode_float(lcmgl, v);
% }
% 
% void bot_lcmgl_enable(bot_lcmgl_t *lcmgl, unsigned int v)
% {
%     bot_lcmgl_encode_u8(lcmgl, BOT_LCMGL_ENABLE);
%     bot_lcmgl_encode_u32(lcmgl, v);
% }
% 
% void bot_lcmgl_disable(bot_lcmgl_t *lcmgl, unsigned int v)
% {
%     bot_lcmgl_encode_u8(lcmgl, BOT_LCMGL_DISABLE);
%     bot_lcmgl_encode_u32(lcmgl, v);
% }
% 
% void bot_lcmgl_materialf(bot_lcmgl_t *lcmgl, int face, int name, float c0,
%                          float c1, float c2,float c3)
% {
%     bot_lcmgl_encode_u8(lcmgl, BOT_LCMGL_MATERIALF);
%     bot_lcmgl_encode_u32(lcmgl, face);
%     bot_lcmgl_encode_u32(lcmgl, name);
%     bot_lcmgl_encode_float(lcmgl, c0);
%     bot_lcmgl_encode_float(lcmgl, c1);
%     bot_lcmgl_encode_float(lcmgl, c2);
%     bot_lcmgl_encode_float(lcmgl, c3);
% }
% 
% void bot_lcmgl_push_attrib(bot_lcmgl_t *lcmgl, unsigned int attrib)
% {
%     bot_lcmgl_encode_u8(lcmgl, BOT_LCMGL_PUSH_ATTRIB);
%     bot_lcmgl_encode_u32(lcmgl, attrib);
% }
% 
% void bot_lcmgl_pop_attrib(bot_lcmgl_t *lcmgl)
% {
%     bot_lcmgl_encode_u8(lcmgl, BOT_LCMGL_POP_ATTRIB);
% }
% 
% void bot_lcmgl_depth_func(bot_lcmgl_t *lcmgl, unsigned int func)
% {
%     bot_lcmgl_encode_u8(lcmgl, BOT_LCMGL_DEPTH_FUNC);
%     bot_lcmgl_encode_u32(lcmgl, func);
% }  
%     

  %% ================ drawing routines not part of OpenGL =============== 

% void bot_lcmgl_box(bot_lcmgl_t *lcmgl, double xyz[3], float size[3])
% {
%     bot_lcmgl_encode_u8(lcmgl, BOT_LCMGL_BOX);
%     bot_lcmgl_encode_double(lcmgl, xyz[0]);
%     bot_lcmgl_encode_double(lcmgl, xyz[1]);
%     bot_lcmgl_encode_double(lcmgl, xyz[2]);
%     bot_lcmgl_encode_float(lcmgl, size[0]);
%     bot_lcmgl_encode_float(lcmgl, size[1]);
%     bot_lcmgl_encode_float(lcmgl, size[2]);
% }
% 
% void bot_lcmgl_circle(bot_lcmgl_t *lcmgl, double xyz[3], double radius)
% {
%     bot_lcmgl_encode_u8(lcmgl, BOT_LCMGL_CIRCLE);
%     bot_lcmgl_encode_double(lcmgl, xyz[0]);
%     bot_lcmgl_encode_double(lcmgl, xyz[1]);
%     bot_lcmgl_encode_double(lcmgl, xyz[2]);
%     bot_lcmgl_encode_float(lcmgl, radius);
% }
% 
% void bot_lcmgl_disk(bot_lcmgl_t *lcmgl, double xyz[3], double r_in,
%         double r_out)
% {
%     bot_lcmgl_encode_u8(lcmgl, BOT_LCMGL_DISK);
%     bot_lcmgl_encode_double(lcmgl, xyz[0]);
%     bot_lcmgl_encode_double(lcmgl, xyz[1]);
%     bot_lcmgl_encode_double(lcmgl, xyz[2]);
%     bot_lcmgl_encode_float(lcmgl, r_in);
%     bot_lcmgl_encode_float(lcmgl, r_out);
% }
% 
% void bot_lcmgl_cylinder(bot_lcmgl_t *lcmgl, double base_xyz[3], double base_radius,
%         double top_radius, double height, int slices, int stacks)
% {
%     bot_lcmgl_encode_u8(lcmgl, BOT_LCMGL_CYLINDER);
%     bot_lcmgl_encode_double(lcmgl, base_xyz[0]);
%     bot_lcmgl_encode_double(lcmgl, base_xyz[1]);
%     bot_lcmgl_encode_double(lcmgl, base_xyz[2]);
%     bot_lcmgl_encode_double(lcmgl, base_radius);
%     bot_lcmgl_encode_double(lcmgl, top_radius);
%     bot_lcmgl_encode_double(lcmgl, height);
%     bot_lcmgl_encode_u32(lcmgl, slices);
%     bot_lcmgl_encode_u32(lcmgl, stacks);
% }
% 

    function bot_lcmgl_sphere(lcmgl, xyz, radius, slices, stacks)
      bot_lcmgl_encode_u8(lcmgl, lcmgl.BOT_LCMGL_SPHERE);
      bot_lcmgl_encode_double(lcmgl, xyz(1));
      bot_lcmgl_encode_double(lcmgl, xyz(2));
      bot_lcmgl_encode_double(lcmgl, xyz(3));
      bot_lcmgl_encode_double(lcmgl, radius);
      bot_lcmgl_encode_u32(lcmgl, slices);
      bot_lcmgl_encode_u32(lcmgl, stacks);
    end
    
%
% void bot_lcmgl_line_width(bot_lcmgl_t *lcmgl, float line_width)
% {
%     bot_lcmgl_encode_u8(lcmgl, BOT_LCMGL_LINE_WIDTH);
%     bot_lcmgl_encode_float(lcmgl, line_width);
% }
% 
% void bot_lcmgl_text_ex(bot_lcmgl_t *lcmgl, const double xyz[3],
%         const char *text, uint32_t font, uint32_t flags)
% {
%     bot_lcmgl_encode_u8(lcmgl, BOT_LCMGL_TEXT_LONG);
%     bot_lcmgl_encode_u32(lcmgl, font);
%     bot_lcmgl_encode_u32(lcmgl, flags);
% 
%     bot_lcmgl_encode_double(lcmgl, xyz[0]);
%     bot_lcmgl_encode_double(lcmgl, xyz[1]);
%     bot_lcmgl_encode_double(lcmgl, xyz[2]);
% 
%     int len = strlen(text);
% 
%     bot_lcmgl_encode_u32(lcmgl, len);
%     for (int i = 0; i < len; i++)
%         bot_lcmgl_encode_u8(lcmgl, text[i]);
% }
% 
% void bot_lcmgl_text(bot_lcmgl_t *lcmgl, const double xyz[3], const char *text)
% {
%     bot_lcmgl_text_ex(lcmgl, xyz, text, 0,
%                  BOT_GL_DRAW_TEXT_DROP_SHADOW |
%                  BOT_GL_DRAW_TEXT_JUSTIFY_CENTER |
%                  BOT_GL_DRAW_TEXT_ANCHOR_HCENTER |
%                  BOT_GL_DRAW_TEXT_ANCHOR_VCENTER);
% }
% 
% 
% void bot_lcmgl_draw_axes(bot_lcmgl_t * lcmgl)
% {
%   //x-axis
%   lcmglBegin(LCMGL_LINES);
%   lcmglColor3f(1, 0, 0);
%   lcmglVertex3f(1, 0, 0);
%   lcmglVertex3f(0, 0, 0);
%   lcmglEnd();
% 
%   //y-axis
%   lcmglBegin(LCMGL_LINES);
%   lcmglColor3f(0, 1, 0);
%   lcmglVertex3f(0, 1, 0);
%   lcmglVertex3f(0, 0, 0);
%   lcmglEnd();
% 
%   //z-axis
%   lcmglBegin(LCMGL_LINES);
%   lcmglColor3f(0, 0, 1);
%   lcmglVertex3f(0, 0, 1);
%   lcmglVertex3f(0, 0, 0);
%   lcmglEnd();
% }
% 
% void
% bot_lcmgl_line(bot_lcmgl_t * lcmgl, double x_start, double y_start, double x_end, double y_end)
% {
%   lcmglBegin(LCMGL_LINES);
%   lcmglVertex2d(x_start, y_start);
%   lcmglVertex2d(x_end, y_end);
%   lcmglEnd();
% }
% 
% void
% bot_lcmgl_draw_ortho_circles_3d(bot_lcmgl_t * lcmgl)
% {
%   double xyz_zero[3] = { 0 };
%   bot_lcmgl_circle(lcmgl, xyz_zero, 1);
%   bot_lcmgl_line(lcmgl, -1, 0, 1, 0);
%   bot_lcmgl_line(lcmgl, 0, -1, 0, 1);
% 
%   lcmglPushMatrix();
%   lcmglRotated(90, 1, 0, 0);
%   bot_lcmgl_circle(lcmgl, xyz_zero, 1);
%   bot_lcmgl_line(lcmgl, -1, 0, 1, 0);
%   bot_lcmgl_line(lcmgl, 0, -1, 0, 1);
%   lcmglPopMatrix();
% 
%   lcmglPushMatrix();
%   lcmglRotated(90, 0, 1, 0);
%   bot_lcmgl_circle(lcmgl, xyz_zero, 1);
%   bot_lcmgl_line(lcmgl, -1, 0, 1, 0);
%   bot_lcmgl_line(lcmgl, 0, -1, 0, 1);
%   lcmglPopMatrix();
% }
% 
% void
% bot_lcmgl_draw_arrow_3d (bot_lcmgl_t * lcmgl, double length, double head_width, double head_length,
%         double body_width)
% {
%     int slices = 20;
%     int stacks = 20;
% 
%     double xyz[3] = {0,0,0};
% 
%     //apply translations so the drawing is centered at origin along the x axis per bot_gl_draw_arrow_2d
%     lcmglPushMatrix();
%     lcmglTranslated(-length / 2, 0, 0);
%     lcmglRotated(90, 0, 1, 0);
% 
%     //draw body
%     lcmglCylinder(xyz, body_width, body_width, length - head_length, slices, stacks);
% 
%     //draw head
%     lcmglTranslated(0, 0, length - head_length);
%     lcmglCylinder(xyz, head_width, 0, head_length, slices, stacks);
% 
%     lcmglPopMatrix();
% }
%   
% void bot_lcmgl_rect(bot_lcmgl_t *lcmgl, double xyz[3], double size[2], int filled)
% {
%     bot_lcmgl_encode_u8(lcmgl, BOT_LCMGL_RECT);
% 
%     bot_lcmgl_encode_double(lcmgl, xyz[0]);
%     bot_lcmgl_encode_double(lcmgl, xyz[1]);
%     bot_lcmgl_encode_double(lcmgl, xyz[2]);
% 
%     bot_lcmgl_encode_double(lcmgl, size[0]);
%     bot_lcmgl_encode_double(lcmgl, size[1]);
% 
%     bot_lcmgl_encode_u8(lcmgl, filled);
% }
% 
% void bot_lcmgl_scale_to_viewer_ar(bot_lcmgl_t *lcmgl){
%   bot_lcmgl_encode_u8(lcmgl, BOT_LCMGL_SCALE_TO_VIEWER_AR);
% }
% 
% 
% // texture API
% 
% int 
% bot_lcmgl_texture2d(bot_lcmgl_t *lcmgl, const void *data, 
%         int width, int height, int row_stride,
%         bot_lcmgl_texture_format_t format,
%         bot_lcmgl_texture_type_t type,
%         bot_lcmgl_compress_mode_t compression)
% {
%     bot_lcmgl_encode_u8(lcmgl, BOT_LCMGL_TEX_2D);
% 
%     uint32_t tex_id = lcmgl->texture_count + 1;
%     lcmgl->texture_count ++;
% 
%     bot_lcmgl_encode_u32(lcmgl, tex_id);
% 
%     int subpix_per_pixel = 1;
%     switch(format) {
%         case BOT_LCMGL_LUMINANCE:
%             subpix_per_pixel = 1;
%             break;
%         case BOT_LCMGL_RGB:
%             subpix_per_pixel = 3;
%             break;
%         case BOT_LCMGL_RGBA:
%             subpix_per_pixel = 4;
%             break;
%     }
% 
%     int bytes_per_subpixel = 1;
%     switch (type) {
%         case BOT_LCMGL_UNSIGNED_BYTE:
%         case BOT_LCMGL_BYTE:
%             bytes_per_subpixel = 1;
%             break;
%         case BOT_LCMGL_UNSIGNED_SHORT:
%         case BOT_LCMGL_SHORT:
%             bytes_per_subpixel = 1;
%             break;
%         case BOT_LCMGL_UNSIGNED_INT:
%         case BOT_LCMGL_INT:
%         case BOT_LCMGL_FLOAT:
%             bytes_per_subpixel = 4;
%             break;
%     }
% 
%     int bytes_per_row = width * subpix_per_pixel * bytes_per_subpixel;
%     int datalen = bytes_per_row * height;
% 
%     bot_lcmgl_encode_u32(lcmgl, width);
%     bot_lcmgl_encode_u32(lcmgl, height);
%     bot_lcmgl_encode_u32(lcmgl, format);
%     bot_lcmgl_encode_u32(lcmgl, type);
%     bot_lcmgl_encode_u32(lcmgl, compression);
% 
%     switch(compression) {
%         case BOT_LCMGL_COMPRESS_NONE:
%             bot_lcmgl_encode_u32(lcmgl, datalen);
%             for(int row=0; row<height; row++) {
%                 void *row_start = (uint8_t*)data + row * row_stride;
%                 bot_lcmgl_encode_raw(lcmgl, bytes_per_row, row_start);
%             }
%             break;
% 
%         case BOT_LCMGL_COMPRESS_ZLIB:
%         {
%           bot_lcmgl_encode_u32(lcmgl, datalen);
%           //compress each row individually
%           uLong uncompressed_size = bytes_per_row;
%           uLong compressed_buf_size = uncompressed_size * 1.01 + 12; //with extra space for zlib
%           Bytef * compressed_buf = (Bytef *) malloc(compressed_buf_size);
%           for(int row=0; row<height; row++) {
%             void *row_start = (uint8_t*)data + row * row_stride;
%             uLong compressed_size = compressed_buf_size;
%             int compress_return = compress2((Bytef *) compressed_buf, &compressed_size, (Bytef *) row_start, uncompressed_size,
%                           Z_BEST_SPEED);
%             if (compress_return != Z_OK) {
%                         fprintf(stderr, "ERROR: Could not compress row %d/%d of texture!\n",row,height);
%                         exit(1);
%                       }
%             bot_lcmgl_encode_u32(lcmgl, compressed_size);
%             bot_lcmgl_encode_raw(lcmgl, compressed_size, compressed_buf);
%           }
%           free(compressed_buf);
%         }
%         break;
%     }
% 
%     return tex_id;
% }
% 
% /**
%  * Renders the specified texture with the active OpenGL color.
%  */
% void 
% bot_lcmgl_texture_draw_quad(bot_lcmgl_t *lcmgl, int texture_id,
%         double x_top_left,  double y_top_left,  double z_top_left,
%         double x_bot_left,  double y_bot_left,  double z_bot_left,
%         double x_bot_right, double y_bot_right, double z_bot_right,
%         double x_top_right, double y_top_right, double z_top_right)
% {
%     if(texture_id > lcmgl->texture_count || texture_id <= 0) {
%         fprintf(stderr, "%s -- WARNING: invalid texture_id %d\n", __FUNCTION__, texture_id);
%         return;
%     }
% 
%     bot_lcmgl_encode_u8(lcmgl, BOT_LCMGL_TEX_DRAW_QUAD);
%     bot_lcmgl_encode_u32(lcmgl, texture_id);
% 
%     bot_lcmgl_encode_double(lcmgl, x_top_left);
%     bot_lcmgl_encode_double(lcmgl, y_top_left);
%     bot_lcmgl_encode_double(lcmgl, z_top_left);
% 
%     bot_lcmgl_encode_double(lcmgl, x_bot_left);
%     bot_lcmgl_encode_double(lcmgl, y_bot_left);
%     bot_lcmgl_encode_double(lcmgl, z_bot_left);
% 
%     bot_lcmgl_encode_double(lcmgl, x_bot_right);
%     bot_lcmgl_encode_double(lcmgl, y_bot_right);
%     bot_lcmgl_encode_double(lcmgl, z_bot_right);
% 
%     bot_lcmgl_encode_double(lcmgl, x_top_right);
%     bot_lcmgl_encode_double(lcmgl, y_top_right);
%     bot_lcmgl_encode_double(lcmgl, z_top_right);
% }
  
  end

  methods 
    function lcmgl = ensure_space(lcmgl, needed)
      if (lcmgl.data_t.datalen + needed < length(lcmgl.data_t.data))
        return;
      end
      
      % grow our buffer.
      lcgml.data_t.data = [lcmgl.data_t.data; 0*lcmgl.data_t.data];
    end
 
    function bot_lcmgl_encode_u8(lcmgl, v)
      ensure_space(lcmgl, 1);
      
      lcmgl.data_t.datalen = lcmgl.data_t.datalen+1;
      lcmgl.data_t.data(lcmgl.data_t.datalen) = uint8(v);
    end
     
    function bot_lcmgl_encode_u32(lcmgl, v)
      v = uint32(v);
      ensure_space(lcmgl, 4);
      
      dl = lcmgl.data_t.datalen;
      lcmgl.data_t.data(dl+1) = mod(bitshift(v,-24),255);
      lcmgl.data_t.data(dl+2) = mod(bitshift(v,-16),255);
      lcgml.data_t.data(dl+3) = mod(bitshift(v,-8),255);
      lcmgl.data_t.data(dl+4) = mod(v,255);
      lcmgl.data_t.datalen = dl+4;
    end
    
    function bot_lcmgl_encode_u64(bot_lcmgl_t *lcmgl, uint64_t v)
      ensure_space(lcmgl, 8);
      
      dl = lcmgl.data_t.datalen;
      lcmgl.data_t.data(dl+1) = mod(bitshift(v,-56),255);
      lcmgl.data_t.data(dl+2) = mod(bitshift(v,-48),255);
      lcmgl.data_t.data(dl+3) = mod(bitshift(v,-40),255);
      lcmgl.data_t.data(dl+4) = mod(bitshift(v,-32),255);
      lcmgl.data_t.data(dl+5) = mod(bitshift(v,-24),255);
      lcmgl.data_t.data(dl+6) = mod(bitshift(v,-16),255);
      lcgml.data_t.data(dl+7) = mod(bitshift(v,-8),255);
      lcmgl.data_t.data(dl+8) = mod(v,255);
      lcmgl.data_t.datalen = dl+8;
    end
 
    function bot_lcmgl_encode_float(lcmgl, f)
      ensure_space(lcmgl, 4);
      
      lcmgl.data_t.data(lcmgl.data_t.datalen+(1:4)) = typecast(uint8(hex2dec(reshape(num2hex(single(f)),2,4)')),'int8');
      lcmgl.data_t.datalen = lcmgl.data_t.datalen+4;
    end
 
    function bot_lcmgl_encode_double(lcmgl, d)
      ensure_space(lcmgl, 8);

      lcmgl.data_t.data(lcmgl.data_t.datalen+(1:8)) = typecast(uint8(hex2dec(reshape(num2hex(d),2,8)')),'int8');
      lcmgl.data_t.datalen = lcmgl.data_t.datalen+8;
    end
 
    function bot_lcmgl_encode_raw(lcmgl, data)
      % not tested yet!
      data = typecast(data,'int8');
      datalen = length(data);
      ensure_space(lcmgl, datalen);
      
      lcmgl.data_t.data(lcmgl.data_t.datalen + (1:datalen)) = data;
      lcmgl.data_t.datalen = lcmgl.data_t.datalen + datalen;
    end
    
    function bot_lcmgl_nop(lcmgl)
      bot_lcmgl_encode_u8(lcmgl, lcmgl.BOT_LCMGL_NOP);
    end
  end
  
  properties (Access=private)
    data_t
    lcm
    channel
  end
  
  properties (Constant) % from lcmgl.h
    LCMGL_POINTS          = uint32(hex2dec('0000'));
    LCMGL_LINES           = uint32(hex2dec('0001'));
    LCMGL_LINE_LOOP       = uint32(hex2dec('0002'));
    LCMGL_LINE_STRIP      = uint32(hex2dec('0003'));
    LCMGL_TRIANGLES       = uint32(hex2dec('0004'));
    LCMGL_TRIANGLE_STRIP  = uint32(hex2dec('0005'));
    LCMGL_TRIANGLE_FAN    = uint32(hex2dec('0006'));
    LCMGL_QUADS           = uint32(hex2dec('0007'));
    LCMGL_QUAD_STRIP      = uint32(hex2dec('0008'));
    LCMGL_POLYGON         = uint32(hex2dec('0009'));
  end

  properties (Constant) % from lcmgl.h enum bot_lcmgl_texture_format_t
    BOT_LCMGL_LUMINANCE = uint32(hex2dec('1909')); %values pulled from gl.h
    BOT_LCMGL_RGB = uint32(hex2dec('1907'));
    BOT_LCMGL_RGBA = uint32(hex2dec('1908'));
  end
  
  properties (Constant) % from lcmgl.h enum bot_lcmgl_texture_type_t
    BOT_LCMGL_UNSIGNED_BYTE = uint32(hex2dec('1401')), % values pulled from gl.h
    BOT_LCMGL_BYTE = uint32(hex2dec('1400')),
    BOT_LCMGL_UNSIGNED_SHORT = uint32(hex2dec('1403')),
    BOT_LCMGL_SHORT = uint32(hex2dec('1402')),
    BOT_LCMGL_UNSIGNED_INT = uint32(hex2dec('1405')),
    BOT_LCMGL_INT = uint32(hex2dec('1404')),
    BOT_LCMGL_FLOAT = uint32(hex2dec('1406'))
  end
  
  properties (Constant) % from lcmgl.h enum bot_lcmgl_compress_mode_t
    BOT_LCMGL_COMPRESS_NONE = 0,
    BOT_LCMGL_COMPRESS_ZLIB = 1,
  end
  
  properties (Constant) % from lcmgl.h enum _bot_lcmgl_enum_t
    BOT_LCMGL_BEGIN          = 4,
    BOT_LCMGL_END            = 5,
    BOT_LCMGL_VERTEX3F       = 6,
    BOT_LCMGL_VERTEX3D       = 7,
    BOT_LCMGL_COLOR3F        = 8,
    BOT_LCMGL_COLOR4F        = 9,
    BOT_LCMGL_POINTSIZE      = 10,
    BOT_LCMGL_ENABLE         = 11,
    BOT_LCMGL_DISABLE        = 12,
    BOT_LCMGL_BOX            = 13,
    BOT_LCMGL_CIRCLE         = 14,
    BOT_LCMGL_LINE_WIDTH     = 15,
    BOT_LCMGL_NOP            = 16,
    BOT_LCMGL_VERTEX2D       = 17,
    BOT_LCMGL_VERTEX2F       = 18,
    BOT_LCMGL_TEXT           = 19,
    BOT_LCMGL_DISK           = 20,
    BOT_LCMGL_TRANSLATED     = 21,
    BOT_LCMGL_ROTATED        = 22,
    BOT_LCMGL_LOAD_IDENTITY  = 23,
    BOT_LCMGL_PUSH_MATRIX    = 24,
    BOT_LCMGL_POP_MATRIX     = 25,
    BOT_LCMGL_RECT           = 26,
    BOT_LCMGL_TEXT_LONG      = 27,
    BOT_LCMGL_NORMAL3F       = 28,
    BOT_LCMGL_SCALEF         = 29,
    BOT_LCMGL_MULT_MATRIXF   = 30,
    BOT_LCMGL_MULT_MATRIXD   = 31,
    BOT_LCMGL_MATERIALF      = 32,
    BOT_LCMGL_PUSH_ATTRIB    = 33,
    BOT_LCMGL_POP_ATTRIB     = 34,
    BOT_LCMGL_DEPTH_FUNC     = 35,
    BOT_LCMGL_TEX_2D         = 36,
    BOT_LCMGL_TEX_DRAW_QUAD  = 37,
    BOT_LCMGL_SPHERE         = 38,
    BOT_LCMGL_CYLINDER       = 39,
    BOT_LCMGL_MATRIX_MODE    = 40,
    BOT_LCMGL_ORTHO          = 41,
    BOT_LCMGL_SCALE_TO_VIEWER_AR = 42
  end
  
  properties (Constant,Access=private)  % from lcmgl.c
    BOT_GL_DRAW_TEXT_DROP_SHADOW    = 1
    BOT_GL_DRAW_TEXT_JUSTIFY_LEFT   = 2
    BOT_GL_DRAW_TEXT_JUSTIFY_RIGHT  = 4
    BOT_GL_DRAW_TEXT_JUSTIFY_CENTER = 8
    BOT_GL_DRAW_TEXT_ANCHOR_LEFT    = 16
    BOT_GL_DRAW_TEXT_ANCHOR_RIGHT   = 32
    BOT_GL_DRAW_TEXT_ANCHOR_TOP     = 64
    BOT_GL_DRAW_TEXT_ANCHOR_BOTTOM  = 128
    BOT_GL_DRAW_TEXT_ANCHOR_HCENTER = 256
    BOT_GL_DRAW_TEXT_ANCHOR_VCENTER = 512
    BOT_GL_DRAW_TEXT_NORMALIZED_SCREEN_COORDINATES = 1024
    BOT_GL_DRAW_TEXT_MONOSPACED     = 2048
  end
end