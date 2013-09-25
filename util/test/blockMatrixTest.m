function test_blockTranspose()
  block_size = [2,1];
  n_block_rows = 1;
  n_block_cols = 2;
  A = [[1;2],[3;4]];
  A_blockwise_transpose = blockwiseTranspose(A,block_size);
  sizecheck(A_blockwise_transpose,[4,1]);
  valuecheck(A_blockwise_transpose,(1:4)');

  A_transposed_blocks = transposeBlocks(A,block_size);
  sizecheck(A_transposed_blocks,[1,4]);
  valuecheck(A_transposed_blocks,1:4);
end
