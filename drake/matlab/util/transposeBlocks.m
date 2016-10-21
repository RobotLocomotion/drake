function A_transposed_blocks = transposeBlocks(A,block_size)
  A_transposed_blocks = blockwiseTranspose(A,block_size)';
end
