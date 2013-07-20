classdef BotLCMGLClient < handle
% intended to be exactly a matlab implementation of the lcmgl.c in
% bot_lcmgl_client

  methods
    function lcmgl = BotLCMGLClient(name)
      checkDependency('lcmgl');
      
      lcmgl.lcm = lcm.lcm.LCM.getSingleton();
      lcmgl.channel = ['LCMGL_',name];
      lcmgl.texture_count = 0;
      
      lcmgl.data_t = bot_lcmgl.data_t();
      lcmgl.data_t.name = name;
      lcmgl.data_t.scene = now;
      lcmgl.data_t.sequence = 0;
      lcmgl.data_t.datalen = 0;
      lcmgl.data_t.data = zeros(1024,1);%zeros(1024*1024,1);
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
      lcmgl.data_t = [];  % this should generate an error if the object is called again
    end
    
  end
  
  methods
    %% ================ OpenGL functions =========== 

    function bot_lcmgl_begin(lcmgl, mode)
      bot_lcmgl_encode_u8(lcmgl, lcmgl.BOT_LCMGL_BEGIN);
      bot_lcmgl_encode_u32(lcmgl, mode);
    end
    
    function bot_lcmgl_end(lcmgl)
      bot_lcmgl_encode_u8(lcmgl, lcmgl.BOT_LCMGL_END);
    end
    
    function bot_lcmgl_vertex2d(lcmgl, v0, v1)
      bot_lcmgl_encode_u8(lcmgl, lcmgl.BOT_LCMGL_VERTEX2D);
      assert (isfinite (v0) && isfinite (v1));
 
      bot_lcmgl_encode_double(lcmgl, v0);
      bot_lcmgl_encode_double(lcmgl, v1);
    end
 
    function bot_lcmgl_vertex2f(lcmgl, v0, v1)
      bot_lcmgl_encode_u8(lcmgl, lcmgl.BOT_LCMGL_VERTEX2F);
      assert (isfinite (v0) && isfinite (v1));
 
      bot_lcmgl_encode_float(lcmgl, v0);
      bot_lcmgl_encode_float(lcmgl, v1);
    end
 
    function bot_lcmgl_vertex3f(lcmgl, v0, v1, v2)
      bot_lcmgl_encode_u8(lcmgl, lcmgl.BOT_LCMGL_VERTEX3F);
      assert (isfinite (v0) && isfinite (v1) && isfinite (v2));
 
      bot_lcmgl_encode_float(lcmgl, v0);
      bot_lcmgl_encode_float(lcmgl, v1);
      bot_lcmgl_encode_float(lcmgl, v2);
    end
    
    function bot_lcmgl_vertex3d(lcmgl, v0, v1, v2)
      bot_lcmgl_encode_u8(lcmgl, lcmgl.BOT_LCMGL_VERTEX3D);
      assert (isfinite (v0) && isfinite (v1) && isfinite (v2));
 
      bot_lcmgl_encode_double(lcmgl, v0);
      bot_lcmgl_encode_double(lcmgl, v1);
      bot_lcmgl_encode_double(lcmgl, v2);
    end
 
    function bot_lcmgl_normal3f(lcmgl, v0, v1, v2)
      bot_lcmgl_encode_u8(lcmgl, lcmgl.BOT_LCMGL_NORMAL3F);
      assert (isfinite (v0) && isfinite (v1) && isfinite (v2));

      bot_lcmgl_encode_float(lcmgl, v0);
      bot_lcmgl_encode_float(lcmgl, v1);
      bot_lcmgl_encode_float(lcmgl, v2);
    end
    
    function bot_lcmgl_scalef(lcmgl, v0, v1, v2)
      bot_lcmgl_encode_u8(lcmgl, lcmgl.BOT_LCMGL_SCALEF);
      assert (isfinite (v0) && isfinite (v1) && isfinite (v2));
      
      bot_lcmgl_encode_float(lcmgl, v0);
      bot_lcmgl_encode_float(lcmgl, v1);
      bot_lcmgl_encode_float(lcmgl, v2);
    end

    function bot_lcmgl_translated(lcmgl, v0, v1, v2)
      bot_lcmgl_encode_u8(lcmgl, lcmgl.BOT_LCMGL_TRANSLATED);
      assert (isfinite (v0) && isfinite (v1) && isfinite (v2));

      bot_lcmgl_encode_double(lcmgl, v0);
      bot_lcmgl_encode_double(lcmgl, v1);
      bot_lcmgl_encode_double(lcmgl, v2);
    end
    
    function bot_lcmgl_rotated(lcmgl, angle, x, y, z)
      bot_lcmgl_encode_u8(lcmgl, lcmgl.BOT_LCMGL_ROTATED);

      bot_lcmgl_encode_double(lcmgl, angle);
      bot_lcmgl_encode_double(lcmgl, x);
      bot_lcmgl_encode_double(lcmgl, y);
      bot_lcmgl_encode_double(lcmgl, z);
    end
    
    function bot_lcmgl_push_matrix (lcmgl)
      bot_lcmgl_encode_u8 (lcmgl, lcmgl.BOT_LCMGL_PUSH_MATRIX);
    end

    function bot_lcmgl_pop_matrix (lcmgl)
      bot_lcmgl_encode_u8 (lcmgl, lcmgl.BOT_LCMGL_POP_MATRIX);
    end

    function bot_lcmgl_mult_matrixf(lcmgl, m)
      bot_lcmgl_encode_u8(lcmgl, lcmgl.BOT_LCMGL_MULT_MATRIXF);

      for i=1:16
        bot_lcmgl_encode_float(lcmgl, m(i));
      end
    end

    function bot_lcmgl_mult_matrixd(lcmgl, m)
      bot_lcmgl_encode_u8(lcmgl, lcmgl.BOT_LCMGL_MULT_MATRIXD);

      for i=1:16
        bot_lcmgl_encode_double(lcmgl, m(i));
      end
    end

    function bot_lcmgl_load_identity(lcmgl)
      bot_lcmgl_encode_u8(lcmgl, lcmgl.BOT_LCMGL_LOAD_IDENTITY);
    end
    
    function bot_lcmgl_matrix_mode(lcmgl, mode)
      bot_lcmgl_encode_u8(lcmgl, lcmgl.BOT_LCMGL_MATRIX_MODE);
      bot_lcmgl_encode_u32(lcmgl, mode);
    end

    function bot_lcmgl_ortho(lcmgl,left,right,bottom,top,nearVal,farVal)
      bot_lcmgl_encode_u8(lcmgl, lcmgl.BOT_LCMGL_ORTHO);
      bot_lcmgl_encode_double(lcmgl, left);
      bot_lcmgl_encode_double(lcmgl, right);
      bot_lcmgl_encode_double(lcmgl, bottom);
      bot_lcmgl_encode_double(lcmgl, top);
      bot_lcmgl_encode_double(lcmgl, nearVal);
      bot_lcmgl_encode_double(lcmgl, farVal);
    end

    function bot_lcmgl_color3f(lcmgl, v0, v1, v2)
      bot_lcmgl_encode_u8(lcmgl, lcmgl.BOT_LCMGL_COLOR3F);
      assert (isfinite (v0) && isfinite (v1) && isfinite (v2));
      bot_lcmgl_encode_float(lcmgl, v0);
      bot_lcmgl_encode_float(lcmgl, v1);
      bot_lcmgl_encode_float(lcmgl, v2);
    end

    function bot_lcmgl_color4f(lcmgl, v0, v1, v2, v3)
      bot_lcmgl_encode_u8(lcmgl, lcmgl.BOT_LCMGL_COLOR4F);
      bot_lcmgl_encode_float(lcmgl, v0);
      bot_lcmgl_encode_float(lcmgl, v1);
      bot_lcmgl_encode_float(lcmgl, v2);
      bot_lcmgl_encode_float(lcmgl, v3);
    end
    
    function bot_lcmgl_point_size(lcmgl, v)
      bot_lcmgl_encode_u8(lcmgl, lcmgl.BOT_LCMGL_POINTSIZE);
      bot_lcmgl_encode_float(lcmgl, v);
    end

    function bot_lcmgl_enable(lcmgl, v)
      bot_lcmgl_encode_u8(lcmgl, lcmgl.BOT_LCMGL_ENABLE);
      bot_lcmgl_encode_u32(lcmgl, v);
    end
    
    function bot_lcmgl_disable(lcmgl, v)
      bot_lcmgl_encode_u8(lcmgl, lcmgl.BOT_LCMGL_DISABLE);
      bot_lcmgl_encode_u32(lcmgl, v);
    end
    
    function bot_lcmgl_materialf(lcmgl,face,name,c0,c1,c2,c3)
      bot_lcmgl_encode_u8(lcmgl, lcmgl.BOT_LCMGL_MATERIALF);
      bot_lcmgl_encode_u32(lcmgl, face);
      bot_lcmgl_encode_u32(lcmgl, name);
      bot_lcmgl_encode_float(lcmgl, c0);
      bot_lcmgl_encode_float(lcmgl, c1);
      bot_lcmgl_encode_float(lcmgl, c2);
      bot_lcmgl_encode_float(lcmgl, c3);
    end
    
    function bot_lcmgl_push_attrib(lcmgl, attrib)
      bot_lcmgl_encode_u8(lcmgl, lcmgl.BOT_LCMGL_PUSH_ATTRIB);
      bot_lcmgl_encode_u32(lcmgl, attrib);
    end

    function bot_lcmgl_pop_attrib(lcmgl)
      bot_lcmgl_encode_u8(lcmgl, lcmgl.BOT_LCMGL_POP_ATTRIB);
    end
    
    function bot_lcmgl_depth_func(lcmgl, func)
      bot_lcmgl_encode_u8(lcmgl, lcmgl.BOT_LCMGL_DEPTH_FUNC);
      bot_lcmgl_encode_u32(lcmgl, func);
    end

    %% ================ drawing routines not part of OpenGL ===============
  
    function bot_lcmgl_box(lcmgl, xyz, size)
      bot_lcmgl_encode_u8(lcmgl, lcmgl.BOT_LCMGL_BOX);
      bot_lcmgl_encode_double(lcmgl, xyz(1));
      bot_lcmgl_encode_double(lcmgl, xyz(2));
      bot_lcmgl_encode_double(lcmgl, xyz(3));
      bot_lcmgl_encode_float(lcmgl, size(1));
      bot_lcmgl_encode_float(lcmgl, size(2));
      bot_lcmgl_encode_float(lcmgl, size(3));
    end
    
    function bot_lcmgl_circle(lcmgl, xyz, radius)
      bot_lcmgl_encode_u8(lcmgl, lcmgl.BOT_LCMGL_CIRCLE);
      bot_lcmgl_encode_double(lcmgl, xyz(1));
      bot_lcmgl_encode_double(lcmgl, xyz(2));
      bot_lcmgl_encode_double(lcmgl, xyz(3));
      bot_lcmgl_encode_float(lcmgl, radius);
    end
    
    function bot_lcmgl_disk(lcmgl, xyz, r_in, r_out)
      bot_lcmgl_encode_u8(lcmgl, lcmgl.BOT_LCMGL_DISK);
      bot_lcmgl_encode_double(lcmgl, xyz(1));
      bot_lcmgl_encode_double(lcmgl, xyz(2));
      bot_lcmgl_encode_double(lcmgl, xyz(3));
      bot_lcmgl_encode_float(lcmgl, r_in);
      bot_lcmgl_encode_float(lcmgl, r_out);
    end
    
    function bot_lcmgl_cylinder(lcmgl, base_xyz, base_radius, top_radius, height, slices, stacks)
      bot_lcmgl_encode_u8(lcmgl, lcmgl.BOT_LCMGL_CYLINDER);
      bot_lcmgl_encode_double(lcmgl, base_xyz(1));
      bot_lcmgl_encode_double(lcmgl, base_xyz(2));
      bot_lcmgl_encode_double(lcmgl, base_xyz(3));
      bot_lcmgl_encode_double(lcmgl, base_radius);
      bot_lcmgl_encode_double(lcmgl, top_radius);
      bot_lcmgl_encode_double(lcmgl, height);
      bot_lcmgl_encode_u32(lcmgl, slices);
      bot_lcmgl_encode_u32(lcmgl, stacks);
    end
    
    function bot_lcmgl_sphere(lcmgl, xyz, radius, slices, stacks)
      bot_lcmgl_encode_u8(lcmgl, lcmgl.BOT_LCMGL_SPHERE);
      bot_lcmgl_encode_double(lcmgl, xyz(1));
      bot_lcmgl_encode_double(lcmgl, xyz(2));
      bot_lcmgl_encode_double(lcmgl, xyz(3));
      bot_lcmgl_encode_double(lcmgl, radius);
      bot_lcmgl_encode_u32(lcmgl, slices);
      bot_lcmgl_encode_u32(lcmgl, stacks);
    end
    
    function bot_lcmgl_line_width(lcmgl, line_width)
      bot_lcmgl_encode_u8(lcmgl, lcmgl.BOT_LCMGL_LINE_WIDTH);
      bot_lcmgl_encode_float(lcmgl, line_width);
    end
    
    function bot_lcmgl_text_ex(lcmgl, xyz, text, font, flags)
      bot_lcmgl_encode_u8(lcmgl, lcmgl.BOT_LCMGL_TEXT_LONG);
      bot_lcmgl_encode_u32(lcmgl, font);
      bot_lcmgl_encode_u32(lcmgl, flags);
      
      bot_lcmgl_encode_double(lcmgl, xyz(1));
      bot_lcmgl_encode_double(lcmgl, xyz(2));
      bot_lcmgl_encode_double(lcmgl, xyz(3));
      
      len = length(text);
      
      bot_lcmgl_encode_u32(lcmgl, len);
      for i=1:len
        bot_lcmgl_encode_u8(lcmgl, text(i));
      end
    end
    
    function bot_lcmgl_text(lcmgl, xyz, text)
      bot_lcmgl_text_ex(lcmgl, xyz, text, 0, ...
        lcmgl.BOT_GL_DRAW_TEXT_DROP_SHADOW + ...
        lcmgl.BOT_GL_DRAW_TEXT_JUSTIFY_CENTER + ...
        lcmgl.BOT_GL_DRAW_TEXT_ANCHOR_HCENTER + ...
        lcmgl.BOT_GL_DRAW_TEXT_ANCHOR_VCENTER);
    end
    
    function bot_lcmgl_draw_axes(lcmgl)
      % x-axis
      bot_lcmgl_begin(lcmgl, lcmgl.LCMGL_LINES);
      bot_lcmgl_color3f(lcmgl, 1, 0, 0);
      bot_lcmgl_vertex3f(lcmgl, 1, 0, 0);
      bot_lcmgl_vertex3f(lcmgl, 0, 0, 0);
      bot_lcmgl_end(lcmgl);
      
      % y-axis
      bot_lcmgl_begin(lcmgl, lcmgl.LCMGL_LINES);
      bot_lcmgl_color3f(lcmgl, 0, 1, 0);
      bot_lcmgl_vertex3f(lcmgl, 0, 1, 0);
      bot_lcmgl_vertex3f(lcmgl, 0, 0, 0);
      bot_lcmgl_end(lcmgl);
      
      % z-axis
      bot_lcmgl_begin(lcmgl, lcmgl.LCMGL_LINES);
      bot_lcmgl_color3f(lcmgl, 0, 0, 1);
      bot_lcmgl_vertex3f(lcmgl, 0, 0, 1);
      bot_lcmgl_vertex3f(lcmgl, 0, 0, 0);
      bot_lcmgl_end(lcmgl);
    end
    
    function bot_lcmgl_line(lcmgl, x_start, y_start, x_end, y_end)
      bot_lcmgl_begin(lcmgl, lcmgl.LCMGL_LINES);
      bot_lcmgl_vertex2d(lcmgl,x_start, y_start);
      bot_lcmgl_vertex2d(lcmgl,x_end, y_end);
      bot_lcmgl_end(lcmgl);
    end

    function bot_lcmgl_draw_ortho_circles_3d(lcmgl)
      xyz_zero = zeros(3,1);
%       bot_lcmgl_color3f(lcmgl, 1, 0, 0);
      bot_lcmgl_circle(lcmgl, xyz_zero, 1);
      bot_lcmgl_line(lcmgl, -1, 0, 1, 0);
      bot_lcmgl_line(lcmgl, 0, -1, 0, 1);
      
%       bot_lcmgl_color3f(lcmgl, 0, 0, 1);
      bot_lcmgl_push_matrix(lcmgl);
      bot_lcmgl_rotated(lcmgl,90, 1, 0, 0);
      bot_lcmgl_circle(lcmgl, xyz_zero, 1);
      bot_lcmgl_line(lcmgl, -1, 0, 1, 0);
      bot_lcmgl_line(lcmgl, 0, -1, 0, 1);
      bot_lcmgl_pop_matrix(lcmgl);
      
%       bot_lcmgl_color3f(lcmgl, 0, 1, 0);
      bot_lcmgl_push_matrix(lcmgl);
      bot_lcmgl_rotated(lcmgl,90, 0, 1, 0);
      bot_lcmgl_circle(lcmgl, xyz_zero, 1);
      bot_lcmgl_line(lcmgl, -1, 0, 1, 0);
      bot_lcmgl_line(lcmgl, 0, -1, 0, 1);
      bot_lcmgl_pop_matrix(lcmgl);
    end
    
    function bot_lcmgl_draw_arrow_3d(lcmgl, length, head_width, head_length, body_width)
      slices = 20;
      stacks = 20;
      
      xyz = zeros(3,1);
      
      % apply translations so the drawing is centered at origin along the x axis per bot_gl_draw_arrow_3d
      bot_lcmgl_push_matrix(lcmgl);
      bot_lcmgl_translated(lcmgl,-length / 2, 0, 0);
      bot_lcmgl_rotated(lcmgl,90,0,1,0);
      
      % draw body
      bot_lcmgl_cylinder(lcmgl, xyz, body_width, body_width, length - head_length, slices, stacks);
      
      % draw head
      bot_lcmgl_translated(lcmgl,0, 0, length - head_length);
      bot_lcmgl_cylinder(lcmgl, xyz, head_width, 0, head_length, slices, stacks);
      
      bot_lcmgl_pop_matrix(lcmgl);
      bot_lcmgl_push_matrix(lcmgl);
    end
    
    function bot_lcmgl_rect(lcmgl, xyz, size, filled)
      bot_lcmgl_encode_u8(lcmgl, lcmgl.BOT_LCMGL_RECT);
      
      bot_lcmgl_encode_double(lcmgl, xyz(1));
      bot_lcmgl_encode_double(lcmgl, xyz(2));
      bot_lcmgl_encode_double(lcmgl, xyz(3));
      
      bot_lcmgl_encode_double(lcmgl, size(1));
      bot_lcmgl_encode_double(lcmgl, size(2));
      
      bot_lcmgl_encode_u8(lcmgl, filled);
    end
    
    function bot_lcmgl_scale_to_viewer_ar(lcmgl)
      bot_lcmgl_encode_u8(lcmgl, lcmgl.BOT_LCMGL_SCALE_TO_VIEWER_AR);
    end
    
    %% texture API
    
    function texture_id = bot_lcmgl_texture2d(lcmgl, data, ...
        width, height, row_stride, ...
        format, type, compression)
      
      error('not implemented yet (just have to translate the code below');
    end
% int 
% bot_lcmgl_texture2d(bot_lcmgl_t *lcmgl, const void *data, 
%         int width, int height, int row_stride,
%         bot_lcmgl_texture_format_t format,
%         bot_lcmgl_texture_type_t type,
%         bot_lcmgl_compress_mode_t compression)
% {
%     bot_lcmgl_encode_u8(lcmgl, lcmgl.BOT_LCMGL_TEX_2D);
% 
%     uint32_t tex_id = lcmgl->texture_count + 1;
%     lcmgl->texture_count ++;
% 
%     bot_lcmgl_encode_u32(lcmgl, tex_id);
% 
%     int subpix_per_pixel = 1;
%     switch(format) {
%         case lcmgl.BOT_LCMGL_LUMINANCE:
%             subpix_per_pixel = 1;
%             break;
%         case lcmgl.BOT_LCMGL_RGB:
%             subpix_per_pixel = 3;
%             break;
%         case lcmgl.BOT_LCMGL_RGBA:
%             subpix_per_pixel = 4;
%             break;
%     }
% 
%     int bytes_per_subpixel = 1;
%     switch (type) {
%         case lcmgl.BOT_LCMGL_UNSIGNED_BYTE:
%         case lcmgl.BOT_LCMGL_BYTE:
%             bytes_per_subpixel = 1;
%             break;
%         case lcmgl.BOT_LCMGL_UNSIGNED_SHORT:
%         case lcmgl.BOT_LCMGL_SHORT:
%             bytes_per_subpixel = 1;
%             break;
%         case lcmgl.BOT_LCMGL_UNSIGNED_INT:
%         case lcmgl.BOT_LCMGL_INT:
%         case lcmgl.BOT_LCMGL_FLOAT:
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
%         case lcmgl.BOT_LCMGL_COMPRESS_NONE:
%             bot_lcmgl_encode_u32(lcmgl, datalen);
%             for(int row=0; row<height; row++) {
%                 void *row_start = (uint8_t*)data + row * row_stride;
%                 bot_lcmgl_encode_raw(lcmgl, bytes_per_row, row_start);
%             }
%             break;
% 
%         case lcmgl.BOT_LCMGL_COMPRESS_ZLIB:
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

    function bot_lcmgl_texture_draw_quad(lcmgl, texture_id, ...
        x_top_left,  y_top_left,  z_top_left, ...
        x_bot_left,  y_bot_left,  z_bot_left, ...
        x_bot_right, y_bot_right, z_bot_right, ...
        x_top_right, y_top_right, z_top_right)
      % Renders the specified texture with the active OpenGL color.

      if(texture_id > lcmgl.texture_count || texture_id <= 0) 
        error('bot_lcmgl_texture_draw_quad -- WARNING: invalid texture_id %d\n', texture_id);
      end
      
      bot_lcmgl_encode_u8(lcmgl, lcmgl.BOT_LCMGL_TEX_DRAW_QUAD);
      bot_lcmgl_encode_u32(lcmgl, texture_id);
      
      bot_lcmgl_encode_double(lcmgl, x_top_left);
      bot_lcmgl_encode_double(lcmgl, y_top_left);
      bot_lcmgl_encode_double(lcmgl, z_top_left);
      
      bot_lcmgl_encode_double(lcmgl, x_bot_left);
      bot_lcmgl_encode_double(lcmgl, y_bot_left);
      bot_lcmgl_encode_double(lcmgl, z_bot_left);
      
      bot_lcmgl_encode_double(lcmgl, x_bot_right);
      bot_lcmgl_encode_double(lcmgl, y_bot_right);
      bot_lcmgl_encode_double(lcmgl, z_bot_right);
      
      bot_lcmgl_encode_double(lcmgl, x_top_right);
      bot_lcmgl_encode_double(lcmgl, y_top_right);
      bot_lcmgl_encode_double(lcmgl, z_top_right);
    end
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
    
    function bot_lcmgl_encode_u64(lcmgl, v)
      v = uint64(v);
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
      f = single(f);
      ensure_space(lcmgl, 4);
      
      lcmgl.data_t.data(lcmgl.data_t.datalen+(1:4)) = typecast(uint8(hex2dec(reshape(num2hex(f),2,4)')),'int8');
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
    texture_count
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
