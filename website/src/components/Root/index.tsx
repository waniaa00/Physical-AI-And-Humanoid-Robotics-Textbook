import React from 'react';
import ChatKit from '../ChatKit';

const Root = ({ children }: { children: React.ReactNode }) => {
  return (
    <>
      {children}
      <ChatKit />
    </>
  );
};

export default Root;